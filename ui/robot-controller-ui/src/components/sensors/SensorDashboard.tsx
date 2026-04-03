// File: src/components/sensors/SensorDashboard.tsx

/**
 * SensorDashboard Component
 *
 * Displays real-time sensor data received over WebSocket:
 * - Line Tracking Sensor (Left / Center / Right + on-line indicator)
 * - Ultrasonic Sensor (distance in cm, m, inches, feet)
 *
 * Applies EMA smoothing and 20 FPS rate-limiting on the ultrasonic channel
 * so the UI stays stable even under rapid sensor updates.
 */

import React, { useEffect, useState, useRef } from 'react';
import { createPortal } from 'react-dom';
import {
  connectLineTrackerWs,
  startJsonHeartbeat,
  parseLineTrackingPayload,
} from '@/utils/connectLineTrackerWs';
import { resolveWsCandidates } from '@/utils/resolveWsUrl';
import { Button } from '@/components/ui/button';
import { Radar } from 'lucide-react';
import UltrasonicVisualization from './UltrasonicVisualization';

// ----------- Types -----------

interface LineTrackingData {
  IR01: number;  // left
  IR02: number;  // center
  IR03: number;  // right
}

interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

// ----------- Constants -----------

/** EMA alpha: higher = more responsive, lower = smoother. */
const EMA_ALPHA = 0.3;
/** Minimum ms between UI updates for ultrasonic (50ms = 20 FPS). */
const UI_INTERVAL_MS = 50;

// ----------- Status dot -----------

function StatusDot({ status, title }: { status: ServerStatus; title: string }) {
  const color =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'connecting'
      ? 'bg-slate-500'
      : 'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
}

// ----------- Line tracker visual indicator -----------

function LineTrackerDots({ data }: { data: LineTrackingData }) {
  const sensors = [
    { label: 'L', active: data.IR01 === 1 },
    { label: 'C', active: data.IR02 === 1 },
    { label: 'R', active: data.IR03 === 1 },
  ];
  const onLine = data.IR01 === 1 || data.IR02 === 1 || data.IR03 === 1;

  return (
    <div className="flex items-center gap-3 mt-2">
      <div className="flex items-center gap-2">
        {sensors.map(({ label, active }) => (
          <div key={label} className="flex flex-col items-center gap-1">
            <span
              className={`w-4 h-4 rounded-full border-2 transition-colors ${
                active
                  ? 'bg-emerald-500 border-emerald-400'
                  : 'bg-gray-700 border-gray-600'
              }`}
            />
            <span className="text-[10px] text-gray-400 font-mono">{label}</span>
          </div>
        ))}
      </div>
      <span
        className={`text-xs font-semibold px-2 py-0.5 rounded ${
          onLine
            ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/40'
            : 'bg-gray-700/40 text-gray-500 border border-gray-700'
        }`}
      >
        {onLine ? 'ON LINE' : 'off line'}
      </span>
    </div>
  );
}

// ----------- Main component -----------

const SensorDashboard: React.FC = () => {
  const [lineTrackingData, setLineTrackingData] = useState<LineTrackingData>({
    IR01: 0,
    IR02: 0,
    IR03: 0,
  });
  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0,
    distance_m: 0,
    distance_inch: 0,
    distance_feet: 0,
  });

  const [lineStatus, setLineStatus] = useState<ServerStatus>('disconnected');
  const [lineLatencyMs, setLineLatencyMs] = useState<number | null>(null);
  const [ultraStatus, setUltraStatus] = useState<ServerStatus>('disconnected');
  const [ultraError, setUltraError] = useState<string | null>(null);
  const [ultraOffline, setUltraOffline] = useState(false);
  const [lineOffline, setLineOffline] = useState(false);
  const [showUltraVisualization, setShowUltraVisualization] = useState(false);

  // WS refs
  const lineWs = useRef<WebSocket | null>(null);
  const stopLineHeartbeat = useRef<null | (() => void)>(null);
  const ultrasonicWs = useRef<WebSocket | null>(null);

  // Smoothing + rate-limit refs (no re-render on write)
  const emaRef = useRef<number | null>(null);        // EMA state; null = uninitialized
  const lastUiUpdateRef = useRef<number>(0);         // epoch ms of last setState call

  // Offline detection: epoch ms of last valid data frame (0 = never)
  const lastUltraDataRef = useRef<number>(0);
  const lastLineDataRef = useRef<number>(0);

  // ── Offline watchdog: check every 200ms, declare offline after 500ms ─
  useEffect(() => {
    const STALE_MS = 500;
    const id = setInterval(() => {
      const now = Date.now();
      setUltraOffline(lastUltraDataRef.current > 0 && now - lastUltraDataRef.current > STALE_MS);
      setLineOffline(lastLineDataRef.current > 0 && now - lastLineDataRef.current > STALE_MS);
    }, 200);
    return () => clearInterval(id);
  }, []);

  // ── Line tracker WS ─────────────────────────────────────────────────
  useEffect(() => {
    let cancelled = false;

    (async () => {
      try {
        setLineStatus('connecting');
        const ws = await connectLineTrackerWs();
        if (cancelled) {
          try { ws.close(); } catch {}
          return;
        }
        lineWs.current = ws;
        setLineStatus('connected');
        setLineLatencyMs(null);

        stopLineHeartbeat.current = startJsonHeartbeat(ws, {
          onLatency: (ms) => setLineLatencyMs(ms),
          onDisconnect: () => setLineStatus('disconnected'),
        });

        ws.addEventListener('message', (event) => {
          try {
            const data = JSON.parse(event.data);

            // Welcome / service-ack
            if (data?.status === 'connected' && data?.service === 'line-tracker') {
              setLineStatus('connected');
              return;
            }

            // Handle normalized blueprint format (from FastAPI /ws/line)
            if (data?.type === 'line_tracking' && data?.left !== undefined) {
              lastLineDataRef.current = Date.now();
              setLineTrackingData({
                IR01: data.left   ? 1 : 0,
                IR02: data.center ? 1 : 0,
                IR03: data.right  ? 1 : 0,
              });
              return;
            }

            // Handle standalone line-tracker server format
            const normalized = parseLineTrackingPayload(data);
            if (normalized) {
              lastLineDataRef.current = Date.now();
              setLineTrackingData(normalized);
            }
          } catch {
            /* ignore */
          }
        });

        ws.onclose = () => setLineStatus('disconnected');
        ws.onerror = () => setLineStatus('disconnected');
      } catch (e) {
        if (!cancelled) {
          console.warn('[LINE] connect failed:', e);
          setLineStatus('disconnected');
        }
      }
    })();

    return () => {
      cancelled = true;
      try { stopLineHeartbeat.current?.(); } catch {}
      try { lineWs.current?.close(); } catch {}
      stopLineHeartbeat.current = null;
      lineWs.current = null;
    };
  }, []);

  // ── Ultrasonic WS ────────────────────────────────────────────────────
  useEffect(() => {
    const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
      defaultPort: '8000',
      path: '/ws/ultrasonic',
    });

    if (!candidates.length) {
      console.error('[ULTRASONIC] No URL candidates. Check NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_* in .env.local');
      setUltraError('No WebSocket URL configured');
      setUltraStatus('disconnected');
      return;
    }

    let candidateIdx = 0;
    let ws: WebSocket | null = null;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    let destroyed = false;

    const tryConnect = () => {
      if (destroyed || candidateIdx >= candidates.length) return;

      const url = candidates[candidateIdx];
      setUltraStatus('connecting');

      try {
        ws = new WebSocket(url);
        ultrasonicWs.current = ws;

        ws.onopen = () => {
          setUltraStatus('connected');
          setUltraError(null);
          emaRef.current = null; // reset smoothing on reconnect
          candidateIdx = 0;
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);

            // Welcome frame — not sensor data
            if (data.type === 'welcome') return;

            // Error frames
            if (data.status === 'error' || data.error) {
              console.warn('[ULTRASONIC] sensor error:', data.error ?? data);
              setUltraError(data.error ?? 'Sensor error');
              return;
            }

            // Sensor reading
            if (data.distance_cm === undefined) return;

            const raw = Number(data.distance_cm);
            if (raw < 2 || raw > 400) return; // reject out-of-range

            // EMA smoothing
            const smoothed =
              emaRef.current === null
                ? raw
                : emaRef.current * (1 - EMA_ALPHA) + raw * EMA_ALPHA;
            emaRef.current = smoothed;

            // Rate-limit UI updates to UI_INTERVAL_MS
            const now = Date.now();
            lastUltraDataRef.current = now;  // stamp on every valid frame
            if (now - lastUiUpdateRef.current < UI_INTERVAL_MS) return;
            lastUiUpdateRef.current = now;

            setUltrasonicData({
              distance_cm:   Math.round(smoothed * 10) / 10,
              distance_m:    Math.round(smoothed / 100 * 1000) / 1000,
              distance_inch: Math.round(smoothed * 0.393701 * 10) / 10,
              distance_feet: Math.round(smoothed * 0.0328084 * 100) / 100,
            });
            setUltraError(null);
          } catch (err) {
            console.error('[ULTRASONIC] parse error:', err);
          }
        };

        ws.onerror = () => {
          setUltraStatus('disconnected');
        };

        ws.onclose = (ev) => {
          if (destroyed) return;
          setUltraStatus('disconnected');
          if (!ev.wasClean && ev.code !== 1000) {
            setUltraError(`Connection closed (code ${ev.code})`);
          }
          // Try next candidate with jitter
          if (candidateIdx < candidates.length - 1) {
            candidateIdx++;
          }
          reconnectTimer = setTimeout(tryConnect, 2000 + Math.random() * 500);
        };
      } catch (err) {
        console.error('[ULTRASONIC] WebSocket creation failed:', err);
        setUltraStatus('disconnected');
        if (!destroyed && candidateIdx < candidates.length - 1) {
          candidateIdx++;
          reconnectTimer = setTimeout(tryConnect, 2000 + Math.random() * 500);
        }
      }
    };

    tryConnect();

    return () => {
      destroyed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      try { ws?.close(); } catch {}
      ultrasonicWs.current = null;
    };
  }, []);

  // ── Tooltip strings ──────────────────────────────────────────────────

  const lineTitle =
    lineStatus === 'connected'
      ? lineLatencyMs != null
        ? `Line tracker: Connected • ${lineLatencyMs}ms`
        : 'Line tracker: Connected'
      : `Line tracker: ${lineStatus[0].toUpperCase()}${lineStatus.slice(1)}`;

  const ultraTitle = ultraError
    ? `Ultrasonic: ${ultraStatus} • ${ultraError}`
    : `Ultrasonic: ${ultraStatus}`;

  // ── Render ───────────────────────────────────────────────────────────

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md w-full max-w-full mx-auto overflow-hidden">
      <h2 className="text-lg font-bold mb-4 text-center">Sensor Dashboard</h2>

      <div className="sensor-container grid grid-cols-1 sm:grid-cols-2 gap-4 lg:gap-6">

        {/* ── Line Tracking ── */}
        <div className="bg-gray-900 p-4 rounded-lg shadow-lg overflow-hidden">
          <div className="flex items-center justify-between mb-2">
            <strong className="text-sm">Line Tracking</strong>
            <div className="flex items-center gap-2">
              {lineOffline && (
                <span className="text-[10px] font-bold uppercase px-1.5 py-0.5 rounded bg-rose-500/20 text-rose-400 border border-rose-500/40 animate-pulse">
                  OFFLINE
                </span>
              )}
              <StatusDot status={lineStatus} title={lineTitle} />
            </div>
          </div>
          <LineTrackerDots data={lineTrackingData} />
          <div className="mt-3 grid grid-cols-3 gap-1 text-xs text-gray-500 font-mono">
            <span>L: {lineTrackingData.IR01}</span>
            <span>C: {lineTrackingData.IR02}</span>
            <span>R: {lineTrackingData.IR03}</span>
          </div>
        </div>

        {/* ── Ultrasonic ── */}
        <div className="bg-gray-900 p-4 rounded-lg shadow-lg overflow-hidden">
          <div className="flex items-center justify-between mb-2">
            <strong className="text-sm">Ultrasonic Distance</strong>
            <div className="flex items-center gap-2">
              {ultraOffline && (
                <span className="text-[10px] font-bold uppercase px-1.5 py-0.5 rounded bg-rose-500/20 text-rose-400 border border-rose-500/40 animate-pulse">
                  OFFLINE
                </span>
              )}
              <StatusDot status={ultraError ? 'disconnected' : ultraStatus} title={ultraTitle} />
            </div>
          </div>

          {ultraError ? (
            <p className="text-xs text-rose-400 italic mt-1">{ultraError}</p>
          ) : (
            <div className="space-y-0.5 mt-1">
              <p className={`text-xl font-bold font-mono ${ultrasonicData.distance_cm === 0 ? 'text-yellow-400' : 'text-white'}`}>
                {ultrasonicData.distance_cm} cm
              </p>
              <p className="text-xs text-gray-400 font-mono">
                {ultrasonicData.distance_m.toFixed(3)} m &nbsp;·&nbsp;
                {ultrasonicData.distance_inch.toFixed(1)} in &nbsp;·&nbsp;
                {ultrasonicData.distance_feet.toFixed(2)} ft
              </p>
            </div>
          )}

          <div className="relative mt-3">
            <Button
              className="w-full gap-2 bg-blue-600 hover:bg-blue-700 text-white"
              variant="default"
              onClick={(e) => {
                e.stopPropagation();
                setShowUltraVisualization((v) => !v);
              }}
            >
              <Radar className="h-4 w-4" />
              {showUltraVisualization ? 'Hide Visualization' : 'Show Visualization'}
            </Button>

            {showUltraVisualization && typeof window !== 'undefined' &&
              createPortal(
                <>
                  <div
                    className="fixed inset-0 bg-black/50 z-[9998]"
                    onClick={() => setShowUltraVisualization(false)}
                  />
                  <div
                    className="fixed top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[380px] max-w-[90vw] overflow-auto z-[9999]"
                    onClick={(e) => e.stopPropagation()}
                    style={{ maxHeight: 'min(85vh, 600px)' }}
                  >
                    <div className="bg-gray-900 border-2 border-green-500 rounded-lg shadow-xl p-2 relative overflow-hidden">
                      <button
                        onClick={() => setShowUltraVisualization(false)}
                        className="absolute top-1 right-1 text-white bg-red-500 hover:bg-red-600 z-[10000] text-xl leading-none w-7 h-7 flex items-center justify-center rounded-full font-bold shadow-lg"
                        aria-label="Close"
                      >
                        ×
                      </button>
                      <UltrasonicVisualization
                        data={ultrasonicData}
                        isConnected={ultraStatus === 'connected'}
                      />
                    </div>
                  </div>
                </>,
                document.body,
              )}
          </div>
        </div>

      </div>
    </div>
  );
};

export default SensorDashboard;
