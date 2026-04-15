// File: src/components/sensors/SensorDashboard.tsx

/**
 * SensorDashboard Component
 *
 * Layout:
 *   ┌──────────────────┬─────────────────────────┐
 *   │  Line Tracking   │  Ultrasonic Distance     │
 *   └──────────────────┴─────────────────────────┘
 *   ┌────────────────────────────────────────────┐
 *   │  Radar visualization (always visible)      │
 *   └────────────────────────────────────────────┘
 *
 * - Line tracking: WS to port 8090 (standalone) or /ws/line (FastAPI)
 * - Ultrasonic: WS to /ws/ultrasonic, with EMA smoothing + 20 FPS rate-limit
 * - Radar: always-visible inline UltrasonicVisualization with sweep controls
 */

import React, { useEffect, useState, useRef } from 'react';
import { createPortal } from 'react-dom';
import {
  connectLineTrackerWs,
  startJsonHeartbeat,
  parseLineTrackingPayload,
} from '@/utils/connectLineTrackerWs';
import { resolveWsCandidates } from '@/utils/resolveWsUrl';
import { Power, Loader2, Radar, X } from 'lucide-react';
import UltrasonicVisualization from './UltrasonicVisualization';

// ----------- Config -----------

const API_BASE =
  (process.env.NEXT_PUBLIC_API_URL ?? '').replace(/\/$/, '') || 'http://localhost:8000';

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

  // Sensor power gate
  const [sensorEnabled, setSensorEnabled] = useState<boolean>(false);
  const [sensorPowerLoading, setSensorPowerLoading] = useState<boolean>(false);
  const [showRadar, setShowRadar] = useState(false);

  // Obstacle avoidance toggle + live state
  const [avoidanceEnabled, setAvoidanceEnabled] = useState<boolean>(false);
  const [avoidanceLoading, setAvoidanceLoading] = useState<boolean>(false);
  const [avoidanceState, setAvoidanceState] = useState<string>('idle');
  const [avoidanceDist,  setAvoidanceDist]  = useState<number | null>(null);
  const [pivotCount,     setPivotCount]     = useState<number>(0);

  // WS refs
  const lineWs = useRef<WebSocket | null>(null);
  const stopLineHeartbeat = useRef<null | (() => void)>(null);
  const ultrasonicWs = useRef<WebSocket | null>(null);

  // Smoothing + rate-limit refs
  const emaRef = useRef<number | null>(null);
  const lastUiUpdateRef = useRef<number>(0);
  const sensorEnabledRef = useRef<boolean>(false);
  const lastUltraDataRef = useRef<number>(0);

  useEffect(() => { sensorEnabledRef.current = sensorEnabled; }, [sensorEnabled]);

  // ── Sensor power + avoidance: fetch state on mount ──────────────────
  useEffect(() => {
    let cancelled = false;
    fetch(`${API_BASE}/api/sensors/power`, { cache: 'no-store' })
      .then(r => r.json())
      .then(d => { if (!cancelled) setSensorEnabled(!!d.enabled); })
      .catch(() => {});
    fetch(`${API_BASE}/autonomy/status`, { cache: 'no-store' })
      .then(r => r.json())
      .then(d => {
        if (!cancelled && d.autonomy) {
          const active = !!d.autonomy.active && d.autonomy.mode === 'avoid_obstacles';
          setAvoidanceEnabled(active);
          if (active && d.autonomy.params) {
            setAvoidanceState(d.autonomy.params.state ?? 'idle');
            setAvoidanceDist(d.autonomy.params.distance_cm ?? null);
            setPivotCount(d.autonomy.params.pivot_count ?? 0);
          }
        }
      })
      .catch(() => {});
    return () => { cancelled = true; };
  }, []);

  // ── Poll /autonomy/status while avoidance is active (2 s interval) ──
  useEffect(() => {
    if (!avoidanceEnabled) return;
    let cancelled = false;
    const poll = () => {
      fetch(`${API_BASE}/autonomy/status`, { cache: 'no-store' })
        .then(r => r.json())
        .then(d => {
          if (cancelled || !d.autonomy) return;
          const active = !!d.autonomy.active && d.autonomy.mode === 'avoid_obstacles';
          setAvoidanceEnabled(active);
          if (d.autonomy.params) {
            setAvoidanceState(d.autonomy.params.state ?? 'idle');
            setAvoidanceDist(d.autonomy.params.distance_cm ?? null);
            setPivotCount(d.autonomy.params.pivot_count ?? 0);
          }
        })
        .catch(() => {});
    };
    const id = setInterval(poll, 2000);
    poll(); // immediate first poll
    return () => { cancelled = true; clearInterval(id); };
  }, [avoidanceEnabled]);

  // ── Avoidance toggle ────────────────────────────────────────────────
  const handleToggleAvoidance = async () => {
    setAvoidanceLoading(true);
    const want = !avoidanceEnabled;
    setAvoidanceEnabled(want);
    try {
      if (want) {
        const res = await fetch(`${API_BASE}/autonomy/start`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            mode: 'avoid_obstacles',
            params: { drive: true, warn_cm: 50, stop_cm: 25 },
          }),
        });
        const d = await res.json();
        const active = d.status === 'ok' && !!d.autonomy?.active;
        setAvoidanceEnabled(active);
        if (active) setAvoidanceState('starting');
      } else {
        const res = await fetch(`${API_BASE}/autonomy/stop`, { method: 'POST' });
        const d = await res.json();
        setAvoidanceEnabled(d.status === 'ok' ? !!d.autonomy?.active : false);
        setAvoidanceState('idle');
        setAvoidanceDist(null);
        setPivotCount(0);
      }
    } catch {
      setAvoidanceEnabled(!want);
    } finally {
      setAvoidanceLoading(false);
    }
  };

  // ── Sensor power toggle ─────────────────────────────────────────────
  const handleToggleSensorPower = async () => {
    setSensorPowerLoading(true);
    const want = !sensorEnabled;
    setSensorEnabled(want);
    if (!want) {
      setUltrasonicData({ distance_cm: 0, distance_m: 0, distance_inch: 0, distance_feet: 0 });
      emaRef.current = null;
    }
    try {
      const res = await fetch(`${API_BASE}/api/sensors/power`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: want }),
      });
      const d = await res.json();
      setSensorEnabled(!!d.enabled);
      if (!d.enabled) {
        setUltrasonicData({ distance_cm: 0, distance_m: 0, distance_inch: 0, distance_feet: 0 });
        emaRef.current = null;
      }
    } catch {
      setSensorEnabled(!want);
    } finally {
      setSensorPowerLoading(false);
    }
  };

  // ── Offline watchdog for ultrasonic ────────────────────────────────
  useEffect(() => {
    const STALE_MS = 500;
    const id = setInterval(() => {
      const now = Date.now();
      const ultraNowOffline = lastUltraDataRef.current > 0 && now - lastUltraDataRef.current > STALE_MS;
      setUltraOffline(prev => {
        if (ultraNowOffline && !prev) {
          setUltrasonicData({ distance_cm: 0, distance_m: 0, distance_inch: 0, distance_feet: 0 });
          emaRef.current = null;
        }
        return ultraNowOffline;
      });
    }, 200);
    return () => clearInterval(id);
  }, []);

  // ── Line tracker WS ─────────────────────────────────────────────────
  useEffect(() => {
    let destroyed = false;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

    const connect = async () => {
      if (destroyed) return;
      try {
        setLineStatus('connecting');
        const ws = await connectLineTrackerWs();
        if (destroyed) { try { ws.close(); } catch {} return; }

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

            if (data?.status === 'connected' && data?.service === 'line-tracker') {
              setLineStatus('connected');
              return;
            }

            if (data?.type === 'line_tracking' && data?.left !== undefined) {
              setLineTrackingData({
                IR01: data.left   ? 1 : 0,
                IR02: data.center ? 1 : 0,
                IR03: data.right  ? 1 : 0,
              });
              return;
            }

            const normalized = parseLineTrackingPayload(data);
            if (normalized) {
              setLineTrackingData(normalized);
            }
          } catch {
            /* ignore */
          }
        });

        ws.onclose = () => {
          setLineStatus('disconnected');
          try { stopLineHeartbeat.current?.(); } catch {}
          stopLineHeartbeat.current = null;
          lineWs.current = null;
          if (!destroyed) reconnectTimer = setTimeout(connect, 3000);
        };
        ws.onerror = () => setLineStatus('disconnected');
      } catch (e) {
        if (!destroyed) {
          console.warn('[LINE] connect failed:', e);
          setLineStatus('disconnected');
          reconnectTimer = setTimeout(connect, 3000);
        }
      }
    };

    connect();

    return () => {
      destroyed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
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
          emaRef.current = null;
          setUltrasonicData({ distance_cm: 0, distance_m: 0, distance_inch: 0, distance_feet: 0 });
          candidateIdx = 0;
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);

            if (data.type === 'welcome') return;

            if (data.status === 'error' || data.error) {
              setUltraError(data.error ?? 'Sensor error');
              return;
            }

            if (data.distance_cm === undefined) return;
            if (!sensorEnabledRef.current) return;

            const raw = Number(data.distance_cm);
            if (raw < 2 || raw > 400) return;

            const smoothed =
              emaRef.current === null
                ? raw
                : emaRef.current * (1 - EMA_ALPHA) + raw * EMA_ALPHA;
            emaRef.current = smoothed;

            const now = Date.now();
            lastUltraDataRef.current = now;
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

        ws.onerror = () => setUltraStatus('disconnected');

        ws.onclose = (ev) => {
          if (destroyed) return;
          setUltraStatus('disconnected');
          setUltrasonicData({ distance_cm: 0, distance_m: 0, distance_inch: 0, distance_feet: 0 });
          emaRef.current = null;
          if (!ev.wasClean && ev.code !== 1000) {
            setUltraError(`Connection closed (code ${ev.code})`);
          }
          if (candidateIdx < candidates.length - 1) candidateIdx++;
          reconnectTimer = setTimeout(tryConnect, 2000 + Math.random() * 500);
        };
      } catch (err) {
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

  // Sensor connected to UI (WS live + not stale)
  const ultraConnectedToUi = ultraStatus === 'connected' && !ultraOffline;

  // Distance color when sensor is on
  const distColor = () => {
    if (!sensorEnabled) return 'text-gray-600';
    if (ultrasonicData.distance_cm === 0) return 'text-yellow-500';
    if (ultrasonicData.distance_cm < 30)  return 'text-red-400';
    if (ultrasonicData.distance_cm < 100) return 'text-yellow-400';
    return 'text-white';
  };

  // ── Render ───────────────────────────────────────────────────────────

  return (
    <div className="bg-gray-800 text-white p-4 rounded-lg shadow-md w-full flex flex-col gap-4">
      <h2 className="text-lg font-bold text-center">Sensor Dashboard</h2>

      {/* ── Top row: two sensor cards side by side ── */}
      <div className="grid grid-cols-2 gap-3">

        {/* ── Line Tracking ── */}
        <div className="bg-gray-900 p-3 rounded-lg shadow-lg flex flex-col gap-1 min-w-0">
          <div className="flex items-center justify-between">
            <strong className="text-xs font-semibold truncate">Line Tracking</strong>
            <StatusDot status={lineStatus} title={lineTitle} />
          </div>
          <LineTrackerDots data={lineTrackingData} />
          <div className="mt-1 grid grid-cols-3 gap-1 text-[10px] text-gray-500 font-mono">
            <span>L:{lineTrackingData.IR01}</span>
            <span>C:{lineTrackingData.IR02}</span>
            <span>R:{lineTrackingData.IR03}</span>
          </div>
        </div>

        {/* ── Ultrasonic Distance ── */}
        <div className="bg-gray-900 p-3 rounded-lg shadow-lg flex flex-col gap-2 min-w-0">
          {/* Header */}
          <div className="flex items-center justify-between gap-1">
            <strong className="text-xs font-semibold truncate">Ultrasonic</strong>
            <div className="flex items-center gap-1.5 shrink-0">
              <button
                onClick={handleToggleSensorPower}
                disabled={sensorPowerLoading}
                className={[
                  'flex items-center gap-0.5 px-1.5 h-5 rounded-full text-[10px] font-bold uppercase',
                  'tracking-wide border transition-all duration-200 select-none',
                  'disabled:cursor-not-allowed disabled:opacity-60',
                  sensorEnabled
                    ? 'bg-rose-600 border-rose-400/80 text-white hover:bg-rose-500 active:scale-95'
                    : 'bg-emerald-900/60 border-emerald-500/50 text-emerald-300 hover:bg-emerald-800/70 active:scale-95',
                ].join(' ')}
                title={sensorEnabled ? 'Disable sensor' : 'Enable sensor'}
              >
                {sensorPowerLoading
                  ? <Loader2 size={8} className="animate-spin" />
                  : <Power size={8} />}
                <span>{sensorEnabled ? 'Off' : 'On'}</span>
              </button>
              {ultraOffline && sensorEnabled && (
                <span className="text-[9px] font-bold px-1 rounded bg-rose-500/20 text-rose-400 border border-rose-500/40 animate-pulse">
                  STALE
                </span>
              )}
              <StatusDot
                status={!sensorEnabled ? 'disconnected' : ultraError ? 'disconnected' : ultraStatus}
                title={ultraTitle}
              />
            </div>
          </div>

          {/* Distance readout — always visible, dimmed when sensor off */}
          <div className={sensorEnabled ? '' : 'opacity-40'}>
            {ultraError && sensorEnabled ? (
              <p className="text-[10px] text-rose-400 font-mono">{ultraError}</p>
            ) : (
              <>
                <p className={`text-2xl font-bold font-mono leading-none ${distColor()}`}>
                  {sensorEnabled ? ultrasonicData.distance_cm.toFixed(0) : '—'}
                  <span className="text-sm font-normal text-gray-400 ml-1">cm</span>
                </p>
                {sensorEnabled && (
                  <p className="text-[10px] text-gray-500 font-mono mt-0.5">
                    {ultrasonicData.distance_m.toFixed(2)}m &nbsp;
                    {ultrasonicData.distance_inch.toFixed(1)}in &nbsp;
                    {ultrasonicData.distance_feet.toFixed(2)}ft
                  </p>
                )}
              </>
            )}
          </div>

          {/* Avoidance toggle */}
          <button
            onClick={handleToggleAvoidance}
            disabled={avoidanceLoading || !sensorEnabled}
            className={[
              'flex items-center justify-center gap-1 w-full h-7 rounded text-[10px] font-bold uppercase',
              'tracking-wide border transition-all duration-200 select-none',
              'disabled:cursor-not-allowed disabled:opacity-50',
              avoidanceEnabled
                ? 'bg-amber-600 border-amber-400/80 text-white hover:bg-amber-500 active:scale-95'
                : 'bg-gray-700/60 border-gray-500/50 text-gray-400 hover:bg-gray-600/70 active:scale-95',
            ].join(' ')}
            title={sensorEnabled ? (avoidanceEnabled ? 'Disable obstacle avoidance' : 'Enable obstacle avoidance') : 'Enable sensor first'}
          >
            {avoidanceLoading && <Loader2 size={8} className="animate-spin" />}
            {avoidanceEnabled ? 'Avoidance ON' : 'Avoidance OFF'}
          </button>

          {/* Avoidance live state strip */}
          {avoidanceEnabled && (() => {
            const stateColors: Record<string, string> = {
              moving:          'text-green-400',
              slowing:         'text-yellow-400',
              pivoting:        'text-orange-400',
              recovering:      'text-sky-400',
              sensor_offline:  'text-rose-400',
              estop:           'text-rose-500',
              error:           'text-rose-500',
            };
            const stateLabels: Record<string, string> = {
              moving:         '▶ MOVING',
              slowing:        '⬇ SLOWING',
              pivoting:       '↺ PIVOTING',
              recovering:     '↻ RECOVERING',
              sensor_offline: '⚠ SENSOR OFFLINE',
              estop:          '⛔ E-STOP',
              error:          '✕ ERROR',
              starting:       '… STARTING',
              idle:           '— IDLE',
            };
            const col = stateColors[avoidanceState] ?? 'text-gray-400';
            const lbl = stateLabels[avoidanceState]  ?? avoidanceState.toUpperCase();
            return (
              <div className="flex items-center justify-between text-[9px] font-mono px-1 mt-0.5">
                <span className={`font-bold ${col}`}>{lbl}</span>
                <span className="text-gray-500">
                  {avoidanceDist != null ? `${avoidanceDist.toFixed(0)} cm` : '—'}
                  {pivotCount > 0 && <span className="ml-1 text-gray-600">×{pivotCount}</span>}
                </span>
              </div>
            );
          })()}
        </div>

      </div>

      {/* ── Radar button ── */}
      <button
        onClick={() => setShowRadar(true)}
        className="w-full flex items-center justify-between gap-3 px-4 py-3
                   bg-gray-900 hover:bg-gray-800 active:scale-[0.99]
                   border border-green-500/25 hover:border-green-500/50
                   rounded-lg transition-all duration-150 group"
      >
        {/* left: icon + label */}
        <div className="flex items-center gap-2">
          <div className="relative">
            <Radar className="h-5 w-5 text-green-400 group-hover:text-green-300 transition-colors" />
            {/* pulse ring when sensor is on */}
            {sensorEnabled && ultraConnectedToUi && (
              <span className="absolute -top-0.5 -right-0.5 w-2 h-2 rounded-full bg-green-400 animate-ping opacity-75" />
            )}
          </div>
          <span className="font-mono font-bold text-sm text-green-400 group-hover:text-green-300 tracking-widest">
            RADAR
          </span>
        </div>

        {/* centre: live distance readout */}
        <span className={`font-mono text-sm font-bold tabular-nums ${distColor()}`}>
          {sensorEnabled && ultrasonicData.distance_cm > 0
            ? `${ultrasonicData.distance_cm.toFixed(0)} cm`
            : '— cm'}
        </span>

        {/* right: open indicator */}
        <span className="text-[10px] font-mono text-gray-500 group-hover:text-gray-400 tracking-wide">
          OPEN ›
        </span>
      </button>

      {/* ── Radar modal (portal) ── */}
      {showRadar && typeof window !== 'undefined' && createPortal(
        <>
          {/* Backdrop */}
          <div
            className="fixed inset-0 bg-black/70 z-[9998] backdrop-blur-sm"
            onClick={() => setShowRadar(false)}
          />

          {/* Panel */}
          <div
            className="fixed top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2
                       w-[min(500px,95vw)] max-h-[90vh] overflow-y-auto
                       z-[9999] rounded-xl shadow-2xl
                       bg-gray-950 border border-green-500/40"
            onClick={e => e.stopPropagation()}
          >
            {/* Modal header bar */}
            <div className="flex items-center justify-between px-4 py-2.5 border-b border-green-500/20">
              <div className="flex items-center gap-2">
                <Radar className="h-4 w-4 text-green-400" />
                <span className="font-mono font-bold text-sm text-green-400 tracking-widest">RADAR</span>
              </div>
              <button
                onClick={() => setShowRadar(false)}
                className="flex items-center justify-center w-6 h-6 rounded-full
                           bg-gray-800 hover:bg-red-600 text-gray-400 hover:text-white
                           transition-colors"
                aria-label="Close radar"
              >
                <X className="h-3.5 w-3.5" />
              </button>
            </div>

            {/* Radar content */}
            <div className="p-3">
              <UltrasonicVisualization
                data={ultrasonicData}
                isConnected={ultraConnectedToUi && sensorEnabled}
              />
            </div>
          </div>
        </>,
        document.body,
      )}

    </div>
  );
};

export default SensorDashboard;
