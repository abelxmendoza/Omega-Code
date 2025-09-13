/*
# File: /src/hooks/useJsonHeartbeat.ts
# Summary:
#   Lightweight JSON heartbeat helper for an existing WebSocket.
#   - Sends periodic {type:"ping", ts} frames and expects {type:"pong", ts?}.
#   - Reports RTT latency (ms) using performance.now() timing.
#   - Clears latency and fires onTimeout() if a pong isn't received in time.
#   - Cleans up timers/listeners on ws change or unmount.
#
#   Options:
#     - intervalMs : ping frequency (default 5000)
#     - timeoutMs  : max wait per ping before considering timed out (default 2500)
#     - onTimeout  : optional callback invoked on pong timeout
*/

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

  useEffect(() => {
    // Stop any previous session
    if (timerRef.current) clearInterval(timerRef.current);
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    timerRef.current = null;
    timeoutRef.current = null;
    setLatencyMs(null);
    pingSentAt.current = null;

    // No socket? Nothing to do.
    if (!ws) return;

    // --- Message handler (expects JSON PONG) ---
    const onMessage = (e: MessageEvent) => {
      try {
        // Some ws impls send non-string data; stringify safely
        const raw = typeof e.data === 'string' ? e.data : '';
        if (!raw) return;

        const msg = JSON.parse(raw);
        if (msg?.type === 'pong') {
          const end = performance.now();
          const start = pingSentAt.current ?? end;
          // Clear the pong deadline if we got a response
          if (timeoutRef.current) {
            clearTimeout(timeoutRef.current);
            timeoutRef.current = null;
          }
          setLatencyMs(Math.max(0, Math.round(end - start)));
          pingSentAt.current = null;
        }
      } catch {
        // Ignore non-JSON or parse errors
      }
    };

    // --- Error/close: clear pending timeout & latency (socket will be managed by owner) ---
    const onError = () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
        timeoutRef.current = null;
      }
      pingSentAt.current = null;
      setLatencyMs(null);
    };
    const onClose = onError;

    ws.addEventListener('message', onMessage);
    ws.addEventListener('error', onError);
    ws.addEventListener('close', onClose);

    // --- Ping tick ---
    const tick = () => {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      try {
        pingSentAt.current = performance.now();
        ws.send(JSON.stringify({ type: 'ping', ts: Date.now() }));

        // Reset pong timeout window
        if (timeoutRef.current) clearTimeout(timeoutRef.current);
        timeoutRef.current = setTimeout(() => {
          // No pong in time: clear state and notify
          pingSentAt.current = null;
          setLatencyMs(null);
          try { onTimeout?.(); } catch { /* ignore user callback errors */ }
        }, timeoutMs);
      } catch {
        // Send errors are non-fatal; next tick will retry
      }
    };

    // Kick once immediately, then schedule
    tick();
    timerRef.current = setInterval(tick, Math.max(250, intervalMs));

    // Cleanup on ws/options change or unmount
    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
      timerRef.current = null;
      timeoutRef.current = null;
      ws.removeEventListener('message', onMessage);
      ws.removeEventListener('error', onError);
      ws.removeEventListener('close', onClose);
    };
  }, [ws, intervalMs, timeoutMs, onTimeout]);

  return { latencyMs };
}
