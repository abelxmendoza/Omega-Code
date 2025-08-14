/*
# File: /Omega-Code/ui/robot-controller-ui/src/hooks/useWsStatus.ts
# Summary:
React hook that tracks a WebSocket's live status per service.
- Supports JSON heartbeat ({type:"ping", ts} → {type:"pong", ts}) and reports RTT latency.
- For streaming-only servers (no pong), `treatAnyMessageAsAlive` keeps the status "connected"
  while messages are flowing and drops to "disconnected" if they stop.
- Auto-reconnects with a short backoff and exposes:
    status: 'connecting' | 'connected' | 'disconnected'
    latency: number | null (ms)
*/

import { useEffect, useRef, useState } from 'react';

export type ServiceStatus = 'connecting' | 'connected' | 'disconnected';

type Options = {
  /** How often to send JSON pings (ms). */
  pingIntervalMs?: number;
  /** How long to wait for a pong before declaring disconnected (ms). */
  pongTimeoutMs?: number;
  /**
   * For servers without pong support (e.g., streaming-only),
   * any inbound message resets the watchdog and marks as connected.
   */
  treatAnyMessageAsAlive?: boolean;
};

export function useWsStatus(url: string | undefined, opts: Options = {}) {
  const {
    pingIntervalMs = 5000,
    pongTimeoutMs = 2500,
    treatAnyMessageAsAlive = false,
  } = opts;

  const [status, setStatus] = useState<ServiceStatus>('connecting');
  const [latency, setLatency] = useState<number | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const pingTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const pongTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    if (!url) {
      setStatus('disconnected');
      setLatency(null);
      return;
    }

    let stopped = false;

    const clearTimers = () => {
      if (pingTimerRef.current) clearInterval(pingTimerRef.current);
      if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
      pingTimerRef.current = null;
      pongTimerRef.current = null;
    };

    const schedulePongWatchdog = () => {
      if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
      pongTimerRef.current = setTimeout(() => {
        setStatus('disconnected');
        try { wsRef.current?.close(); } catch {}
      }, pongTimeoutMs);
    };

    const connect = () => {
      setStatus('connecting');
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        if (stopped) return;

        // Start heartbeat loop
        const sendPing = () => {
          if (ws.readyState !== WebSocket.OPEN) return;
          const ts = Date.now();
          try {
            ws.send(JSON.stringify({ type: 'ping', ts }));
          } catch {}
          schedulePongWatchdog();
        };

        // kick off immediately, then every pingIntervalMs
        sendPing();
        pingTimerRef.current = setInterval(sendPing, pingIntervalMs);
      };

      ws.onmessage = (e) => {
        if (stopped) return;
        try {
          const msg = JSON.parse(e.data);
          if (msg?.type === 'pong' && typeof msg.ts === 'number') {
            if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
            setLatency(Math.max(0, Date.now() - msg.ts));
            setStatus('connected');
            return;
          }
        } catch {
          // non-JSON; fall through to streaming handling if enabled
        }

        if (treatAnyMessageAsAlive) {
          if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
          setStatus('connected');
          // If messages stop arriving, watchdog will flip to disconnected
          schedulePongWatchdog();
        }
      };

      ws.onclose = () => {
        if (stopped) return;
        setStatus('disconnected');
        setLatency(null);
        clearTimers();
        // auto-reconnect
        setTimeout(connect, 1500);
      };

      ws.onerror = () => {
        // force close to trigger reconnect path
        try { ws.close(); } catch {}
      };
    };

    connect();

    return () => {
      stopped = true;
      clearTimers();
      try { wsRef.current?.close(); } catch {}
    };
  }, [url, pingIntervalMs, pongTimeoutMs, treatAnyMessageAsAlive]);

  return { status, latency };
}
