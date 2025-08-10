import { useEffect, useRef, useState } from 'react';

export type ServiceStatus = 'connecting' | 'connected' | 'disconnected';

type Options = {
  pingIntervalMs?: number;     // how often to ping
  pongTimeoutMs?: number;      // how long to wait for pong
  treatAnyMessageAsAlive?: boolean; // for servers without pong, any message resets the watchdog
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
  const lastPingTsRef = useRef<number>(0);

  useEffect(() => {
    if (!url) {
      setStatus('disconnected');
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
        setStatus('connected');

        // heartbeat loop
        const sendPing = () => {
          if (ws.readyState !== WebSocket.OPEN) return;
          const ts = Date.now();
          lastPingTsRef.current = ts;
          try {
            ws.send(JSON.stringify({ type: 'ping', ts }));
          } catch {}
          schedulePongWatchdog();
        };

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
          } else if (treatAnyMessageAsAlive) {
            // For servers that stream data instead of pong (e.g., ultrasonic):
            if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
            setStatus('connected');
            // restart watchdog so if messages stop, we drop to disconnected
            schedulePongWatchdog();
          }
        } catch {
          // Non-JSON? Count it as alive for streaming endpoints
          if (treatAnyMessageAsAlive) {
            if (pongTimerRef.current) clearTimeout(pongTimerRef.current);
            setStatus('connected');
            schedulePongWatchdog();
          }
        }
      };

      ws.onclose = () => {
        if (stopped) return;
        setStatus('disconnected');
        clearTimers();
        // auto-reconnect
        setTimeout(connect, 1500);
      };

      ws.onerror = () => {
        // force close to trigger reconnect
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
