/**
 * File: src/components/sensors/LineTrackerStatus.tsx
 * Summary:
 *   Real-time status panel for the robot's line tracking sensor (3 IR sensors).
 *   Connects to a WebSocket server (URL from .env only) and displays IR01, IR02, IR03 state.
 *   Compatible with { lineTracking: { left, center, right } }, { sensors: {...} }, or { IR01, IR02, IR03 } JSON.
 *   Shows a connection pill (connecting/connected/disconnected) with optional latency from JSON ping/pong.
 */

'use client';

import React, { useState, useEffect, useRef, useCallback } from 'react';

// Prefer Tailscale, fallback to LAN. Never hardcode your own IP!
const LINE_TRACKER_WS =
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN ||
  'ws://localhost:3001/ws/line';

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

const LineTrackerStatus: React.FC = () => {
  const [data, setData] = useState({ IR01: 0, IR02: 0, IR03: 0 });
  const [status, setStatus] = useState<ServerStatus>('disconnected');
  const [latencyMs, setLatencyMs] = useState<number | null>(null);

  const ws = useRef<WebSocket | null>(null);
  const hbTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const pongTimeout = useRef<ReturnType<typeof setTimeout> | null>(null);
  const pingSentAt = useRef<number | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const mounted = useRef(false);

  const statusColor =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'connecting'
      ? 'bg-slate-600'
      : 'bg-rose-600';

  const statusLabel =
    status === 'connected'
      ? latencyMs != null
        ? `Connected • ${latencyMs}ms`
        : 'Connected'
      : status === 'connecting'
      ? 'Connecting…'
      : 'Disconnected';

  // Normalize payload from various backends
  const normalize = (raw: any) => {
    if (raw?.lineTracking) {
      const lt = raw.lineTracking;
      return {
        IR01: Number(lt.left ?? 0),
        IR02: Number(lt.center ?? 0),
        IR03: Number(lt.right ?? 0),
      };
    }
    if (raw?.sensors) {
      const s = raw.sensors;
      return {
        IR01: Number(s.left ?? 0),
        IR02: Number(s.center ?? 0),
        IR03: Number(s.right ?? 0),
      };
    }
    if (
      raw?.IR01 !== undefined &&
      raw?.IR02 !== undefined &&
      raw?.IR03 !== undefined
    ) {
      return {
        IR01: Number(raw.IR01),
        IR02: Number(raw.IR02),
        IR03: Number(raw.IR03),
      };
    }
    return null;
  };

  const clearHeartbeat = useCallback(() => {
    if (hbTimer.current) {
      clearInterval(hbTimer.current);
      hbTimer.current = null;
    }
    if (pongTimeout.current) {
      clearTimeout(pongTimeout.current);
      pongTimeout.current = null;
    }
    pingSentAt.current = null;
  }, []);

  const startHeartbeat = useCallback(() => {
    clearHeartbeat();
    hbTimer.current = setInterval(() => {
      if (!ws.current || ws.current.readyState !== WebSocket.OPEN) return;
      try {
        // Send JSON ping
        pingSentAt.current = performance.now();
        ws.current.send(JSON.stringify({ type: 'ping', ts: Date.now() }));

        // Reset/arm pong timeout
        if (pongTimeout.current) clearTimeout(pongTimeout.current);
        pongTimeout.current = setTimeout(() => {
          // No pong in time → drop latency but keep connection state
          pingSentAt.current = null;
          if (mounted.current) setLatencyMs(null);
        }, 6000);
      } catch {
        if (mounted.current) setStatus('disconnected');
      }
    }, 10000);
  }, [clearHeartbeat]);

  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);

  useEffect(() => {
    if (!LINE_TRACKER_WS) return;

    let cancelled = false;

    const openWs = (attempt = 0) => {
      if (cancelled) return;

      if (mounted.current) setStatus('connecting');

      try {
        ws.current = new WebSocket(LINE_TRACKER_WS);
      } catch {
        // Schedule reconnect on constructor failure
        const backoff = Math.min(1000 * Math.pow(2, attempt), 10000);
        reconnectTimer.current = setTimeout(() => openWs(attempt + 1), backoff);
        return;
      }

      ws.current.onopen = () => {
        if (!mounted.current) return;
        setStatus('connected');
        setLatencyMs(null);
        startHeartbeat();
        // console.log('[LINE TRACKER] WebSocket connection established');
      };

      ws.current.onmessage = (event) => {
        if (!mounted.current) return;
        try {
          const result = JSON.parse(event.data);

          // Welcome envelope
          if (result?.status === 'connected' && result?.service === 'line-tracker') {
            setStatus('connected');
            return;
          }

          // Pong
          if (result?.type === 'pong') {
            const end = performance.now();
            const start = pingSentAt.current ?? end;
            if (pongTimeout.current) {
              clearTimeout(pongTimeout.current);
              pongTimeout.current = null;
            }
            setLatencyMs(Math.max(0, Math.round(end - start)));
            pingSentAt.current = null;
            return;
          }

          const normalized = normalize(result);
          if (normalized) setData(normalized);
        } catch {
          /* ignore invalid JSON */
        }
      };

      ws.current.onerror = () => {
        // onclose will handle reconnect; no-op here
      };

      ws.current.onclose = () => {
        if (!mounted.current) return;
        setStatus('disconnected');
        clearHeartbeat();
        const backoff = Math.min(1000 * Math.pow(2, attempt), 10000);
        reconnectTimer.current = setTimeout(() => openWs(attempt + 1), backoff);
        // console.log('[LINE TRACKER] WebSocket connection closed');
      };
    };

    openWs(0);

    return () => {
      cancelled = true;
      clearHeartbeat();
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
      if (ws.current) {
        ws.current.onopen = null;
        ws.current.onclose = null;
        ws.current.onerror = null;
        ws.current.onmessage = null;
        try { ws.current.close(); } catch {}
        ws.current = null;
      }
    };
  }, [startHeartbeat, clearHeartbeat]); // include memoized heartbeat

  return (
    <div className="bg-gray-900 text-white p-4 rounded-lg shadow-md my-2 relative">
      {/* Status pill */}
      <div className={`absolute top-3 right-3 px-2 py-1 rounded-full text-xs flex items-center gap-2 ${statusColor}`}>
        <span className="inline-block w-2 h-2 rounded-full bg-white/90" />
        <span className="font-semibold">{statusLabel}</span>
      </div>

      <h3 className="text-md font-semibold mb-2 underline">Line Tracking Status</h3>
      <p>IR01: {data.IR01}</p>
      <p>IR02: {data.IR02}</p>
      <p>IR03: {data.IR03}</p>
    </div>
  );
};

export default LineTrackerStatus;
