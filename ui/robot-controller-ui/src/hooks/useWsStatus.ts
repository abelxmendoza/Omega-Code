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
- NEW: `enabled?: boolean` to allow callers to always mount the hook but no-op when disabled.
*/

import { useEffect, useRef, useState } from 'react';

export type ServiceStatus = 'connecting' | 'connected' | 'disconnected';

type Options = {
  /** When false, the hook cleans up and reports 'disconnected' without opening a socket. */
  enabled?: boolean;
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

type Result = { status: ServiceStatus; latency: number | null };

export function useWsStatus(url: string | undefined, opts: Options = {}): Result {
  const {
    enabled = true,
    pingIntervalMs = 5000,
    pongTimeoutMs = 2500,
    treatAnyMessageAsAlive = false,
  } = opts;

  const [status, setStatus] = useState<ServiceStatus>('connecting');
  const [latency, setLatency] = useState<number | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const pingTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const pongTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  // Helpers
  const clearTimers = () => {
    if (pingTimerRef.current) clearInterval(pingTimerRef.current);
    if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
    if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
    pingTimerRef.current = null;
    pongTimerRef.current = null;
    reconnectTimerRef.current = null;
  };

  useEffect(() => {
    // If disabled or URL missing, fully clean up and expose a stable disconnected state.
    if (!enabled || !url) {
      clearTimers();
      try { wsRef.current?.close(); } catch {}
      wsRef.current = null;
      setStatus('disconnected');
      setLatency(null);
      return;
    }

    let stopped = false;

    const schedulePongWatchdog = () => {
      if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
      pongTimerRef.current = setTimeout(() => {
        // No pong (or messages in streaming mode) within timeout → drop to disconnected
        setStatus('disconnected');
        try { wsRef.current?.close(); } catch {}
      }, pongTimeoutMs);
    };

    const connect = () => {
      if (stopped) return;

      setStatus('connecting');
      setLatency(null);

      // Clear any previous timers before a fresh connection
      clearTimers();

      const ws = new WebSocket(url);
      wsRef.current = ws;

      let lastPingTs = 0;

      const sendPing = () => {
        if (ws.readyState !== WebSocket.OPEN) return;
        const ts = Date.now();
        lastPingTs = ts;
        try {
          ws.send(JSON.stringify({ type: 'ping', ts }));
        } catch {}
        schedulePongWatchdog();
      };

      ws.onopen = () => {
        if (stopped) return;
        setStatus('connected');
        // Start heartbeat immediately, then repeat
        sendPing();
        pingTimerRef.current = setInterval(sendPing, pingIntervalMs);
      };

      ws.onmessage = (e) => {
        if (stopped) return;

        // Try JSON first for pong
        let parsed: any = null;
        try {
          parsed = JSON.parse(e.data);
        } catch {
          // non-JSON; ignore parse error
        }

        if (parsed && parsed.type === 'pong') {
          if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
          setStatus('connected');
          // Prefer returned ts; fall back to lastPingTs if absent
          const sentTs = typeof parsed.ts === 'number' ? parsed.ts : lastPingTs;
          if (sentTs) setLatency(Math.max(0, Date.now() - sentTs));
          return;
        }

        // Streaming-alive mode: any message counts as life-sign
        if (treatAnyMessageAsAlive) {
          if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
          setStatus('connected');
          // Re-arm watchdog so silence will flip to disconnected
          schedulePongWatchdog();
        }
      };

      ws.onerror = () => {
        // Let onclose handle the reconnect & state; forcing close ensures we go through that path
        try { ws.close(); } catch {}
      };

      ws.onclose = () => {
        if (stopped) return;
        setStatus('disconnected');
        setLatency(null);
        clearTimers();
        // Auto-reconnect with a small backoff
        reconnectTimerRef.current = setTimeout(() => {
          if (!stopped) connect();
        }, 1500);
      };
    };

    connect();

    return () => {
      stopped = true;
      clearTimers();
      try { wsRef.current?.close(); } catch {}
      wsRef.current = null;
    };
  }, [enabled, url, pingIntervalMs, pongTimeoutMs, treatAnyMessageAsAlive]);

  return { status, latency };
}
