// src/hooks/useJsonHeartbeat.ts
'use client';
import { useEffect, useRef, useState } from 'react';

type Options = {
  intervalMs?: number;
  timeoutMs?: number;
  onTimeout?: () => void; // optional: called when a ping doesn't get a pong in time
};

export function useJsonHeartbeat(ws: WebSocket | null, opts: Options = {}) {
  const { intervalMs = 5000, timeoutMs = 2500, onTimeout } = opts;
  const [latencyMs, setLatencyMs] = useState<number | null>(null);

  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const timeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const pingSentAt = useRef<number | null>(null);
  const listenerRef = useRef<(e: MessageEvent) => void | null>(null);

  useEffect(() => {
    // stop any previous session
    if (timerRef.current) clearInterval(timerRef.current);
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    setLatencyMs(null);
    pingSentAt.current = null;

    if (!ws) return;

    // listen for pong
    const onMessage = (e: MessageEvent) => {
      try {
        const msg = JSON.parse(e.data);
        if (msg?.type === 'pong') {
          const end = performance.now();
          const start = pingSentAt.current ?? end;
          if (timeoutRef.current) clearTimeout(timeoutRef.current);
          timeoutRef.current = null;
          setLatencyMs(Math.max(0, Math.round(end - start)));
          pingSentAt.current = null;
        }
      } catch {
        /* ignore non-JSON */
      }
    };
    ws.addEventListener('message', onMessage);
    listenerRef.current = onMessage;

    const tick = () => {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      try {
        pingSentAt.current = performance.now();
        ws.send(JSON.stringify({ type: 'ping', ts: Date.now() }));
        // pong timeout
        if (timeoutRef.current) clearTimeout(timeoutRef.current);
        timeoutRef.current = setTimeout(() => {
          pingSentAt.current = null;
          setLatencyMs(null);
          onTimeout?.();
        }, timeoutMs);
      } catch {
        // ignore send failures; next tick will retry
      }
    };

    // kick + schedule
    tick();
    timerRef.current = setInterval(tick, intervalMs);

    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
      if (listenerRef.current) ws.removeEventListener('message', listenerRef.current);
      timerRef.current = null;
      timeoutRef.current = null;
      listenerRef.current = null;
    };
  }, [ws, intervalMs, timeoutMs, onTimeout]);

  return { latencyMs };
}
