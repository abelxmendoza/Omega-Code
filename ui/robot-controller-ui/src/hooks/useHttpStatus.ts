// /src/hooks/useHttpStatus.ts
import { useEffect, useRef, useState } from 'react';

export type HttpStatus = 'connected' | 'connecting' | 'disconnected';

type Options = {
  intervalMs?: number;   // how often to probe
  timeoutMs?: number;    // per-probe timeout
};

export function useHttpStatus(url: string | undefined, opts: Options = {}) {
  const { intervalMs = 5000, timeoutMs = 2500 } = opts;
  const [status, setStatus] = useState<HttpStatus>('connecting');
  const [latency, setLatency] = useState<number | null>(null);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    if (!url) {
      setStatus('disconnected');
      setLatency(null);
      return;
    }

    const probe = async () => {
      setStatus(s => (s === 'connected' ? s : 'connecting'));
      const start = performance.now();
      const ctrl = new AbortController();
      const t = setTimeout(() => ctrl.abort(), timeoutMs);

      try {
        // no-cors => opaque response but resolves if reachable
        await fetch(url, { mode: 'no-cors', cache: 'no-store', signal: ctrl.signal });
        const ms = Math.round(performance.now() - start);
        setLatency(ms);
        setStatus('connected');
      } catch {
        setLatency(null);
        setStatus('disconnected');
      } finally {
        clearTimeout(t);
      }
    };

    // immediate probe, then interval
    probe();
    if (timerRef.current) clearInterval(timerRef.current);
    timerRef.current = setInterval(probe, intervalMs);

    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
    };
  }, [url, intervalMs, timeoutMs]);

  return { status, latency };
}
