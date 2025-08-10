// File: src/components/sensors/SensorDashboard.tsx

/**
 * SensorDashboard Component
 *
 * Displays real-time sensor data received over WebSocket:
 * - Line Tracking Sensor (IR01, IR02, IR03)
 * - Ultrasonic Sensor (distance in cm, m, inches, feet)
 *
 * Tries Tailscale first, falls back to LAN. Works with both
 * separate or combined servers for line tracking & ultrasonic.
 */

import React, { useEffect, useState, useRef } from 'react';
import {
  connectLineTrackerWs,
  startJsonHeartbeat,
  parseLineTrackingPayload,
} from '@/utils/connectLineTrackerWs';
import { resolveWsUrl } from '@/utils/resolveWsUrl';

interface LineTrackingData {
  IR01: number;
  IR02: number;
  IR03: number;
}
interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

// ------------- WebSocket URLs (from env) ----------------
const ULTRASONIC_WS = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');

// Small status dot helper
function StatusDot({
  status,
  title,
}: {
  status: ServerStatus;
  title: string;
}) {
  const color =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'connecting'
      ? 'bg-slate-500'
      : 'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }} // smaller than w-2/h-2 (8px)
      title={title}
      aria-label={title}
    />
  );
}

const SensorDashboard: React.FC = () => {
  // --- State ---
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

  // Connection statuses
  const [lineStatus, setLineStatus] = useState<ServerStatus>('disconnected');
  const [lineLatencyMs, setLineLatencyMs] = useState<number | null>(null);
  const [ultraStatus, setUltraStatus] = useState<ServerStatus>('disconnected');

  // --- WebSocket refs ---
  const lineWs = useRef<WebSocket | null>(null);
  const stopLineHeartbeat = useRef<null | (() => void)>(null);
  const ultrasonicWs = useRef<WebSocket | null>(null);

  // Line tracker WS (via helper)
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

        // Start JSON ping/pong heartbeat
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
            const normalized = parseLineTrackingPayload(data);
            if (normalized) setLineTrackingData(normalized);
          } catch {
            /* ignore */
          }
        });

        ws.onclose = () => setLineStatus('disconnected');
        ws.onerror = () => setLineStatus('disconnected');
      } catch (e) {
        console.warn('[LINE] connect failed:', e);
        setLineStatus('disconnected');
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

  // Ultrasonic WS (direct)
  useEffect(() => {
    if (!ULTRASONIC_WS) return;

    setUltraStatus('connecting');
    ultrasonicWs.current = new WebSocket(ULTRASONIC_WS);

    ultrasonicWs.current.onopen = () => {
      setUltraStatus('connected');
      console.log(`[ULTRASONIC] WebSocket connected: ${ULTRASONIC_WS}`);
    };

    ultrasonicWs.current.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.distance_cm !== undefined) {
          setUltrasonicData({
            distance_cm: Number(data.distance_cm ?? 0),
            distance_m: Number(data.distance_m ?? 0),
            distance_inch: Number(data.distance_inch ?? 0),
            distance_feet: Number(data.distance_feet ?? 0),
          });
        }
      } catch {
        /* ignore */
      }
    };

    ultrasonicWs.current.onerror = (e) => {
      console.warn('[ULTRASONIC] WebSocket error:', e);
      setUltraStatus('disconnected');
    };
    ultrasonicWs.current.onclose = () => {
      console.log('[ULTRASONIC] WebSocket closed');
      setUltraStatus('disconnected');
    };

    return () => ultrasonicWs.current?.close();
  }, []);

  // Tooltip strings
  const lineTitle =
    lineStatus === 'connected'
      ? lineLatencyMs != null
        ? `Line tracker: Connected • ${lineLatencyMs}ms`
        : 'Line tracker: Connected'
      : `Line tracker: ${lineStatus[0].toUpperCase()}${lineStatus.slice(1)}`;

  const ultraTitle = `Ultrasonic: ${ultraStatus[0].toUpperCase()}${ultraStatus.slice(1)}`;

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md max-w-6xl mx-auto">
      <h2 className="text-lg font-bold mb-4 text-center">Sensor Dashboard</h2>

      <div className="sensor-container grid grid-cols-1 sm:grid-cols-2 gap-6">
        {/* Line Tracking */}
        <div className="line-tracking bg-gray-900 p-4 rounded-lg shadow-lg">
          <div className="flex items-center justify-between mb-2">
            <strong className="block text-sm underline">Line Tracking Sensor</strong>
            <StatusDot status={lineStatus} title={lineTitle} />
          </div>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>

        {/* Ultrasonic */}
        <div className="ultrasonic-distance bg-gray-900 p-4 rounded-lg shadow-lg">
          <div className="flex items-center justify-between mb-2">
            <strong className="block text-sm underline">Ultrasonic Distance</strong>
            <StatusDot status={ultraStatus} title={ultraTitle} />
          </div>
          <p>{ultrasonicData.distance_cm} cm</p>
          <p>{ultrasonicData.distance_m.toFixed(2)} m</p>
          <p>{ultrasonicData.distance_inch.toFixed(2)} inches</p>
          <p>{ultrasonicData.distance_feet.toFixed(2)} feet</p>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;
