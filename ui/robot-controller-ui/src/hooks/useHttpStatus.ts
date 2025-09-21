/*
# File: /Omega-Code/ui/robot-controller-ui/src/hooks/useHttpStatus.ts
# Summary:
#   Lightweight HTTP status probe for service health endpoints (e.g., /health).
#   - Uses GET so we can read the JSON body and map states precisely:
#       • 200                        → "connected"
#       • 503 + { placeholder:true } → "no_camera"
#       • 503 (other)                → "connecting"  (server reachable but not ready)
#       • network/CORS errors        → "disconnected"
#
#   Options:
#     - enabled   : when false, do nothing and report "disconnected" (default true)
#     - intervalMs: poll frequency (default 5000)
#     - timeoutMs : request timeout (default 2500)
#     - treat503AsConnecting : map generic 503 to "connecting" (default true)
*/

import { useEffect, useRef, useState } from 'react';

export type HttpStatus = 'connected' | 'connecting' | 'disconnected' | 'no_camera';

export interface HttpState {
  status: HttpStatus;
  latency: number | null;
}

type Options = {
  enabled?: boolean;              // gate all work; stable disconnected state when false
  intervalMs?: number;            // how often to probe
  timeoutMs?: number;             // per-probe timeout
  treat503AsConnecting?: boolean; // map raw 503 to "connecting"
};

export function useHttpStatus(url: string | undefined, opts: Options = {}): HttpState {
  const {
    enabled = true,
    intervalMs = 5000,
    timeoutMs = 2500,
    treat503AsConnecting = true,
  } = opts;

  const [status, setStatus] = useState<HttpStatus>('disconnected');
  const [latency, setLatency] = useState<number | null>(null);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    // Always clear any existing timer when inputs change
    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }

    // Disabled or no URL → stable disconnected, no network activity
    if (!enabled || !url) {
      setStatus('disconnected');
      setLatency(null);
      return;
    }

    let cancelled = false;

    const probe = async () => {
      // Only escalate to "connecting" if we aren't already in a good known state
      setStatus((s) => (s === 'connected' || s === 'no_camera' ? s : 'connecting'));

      const start = performance.now();
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), timeoutMs);

      try {
        // Use GET with CORS so we can read JSON body from /health
        const res = await fetch(url, {
          method: 'GET',
          cache: 'no-store',
          mode: 'cors' as RequestMode,
          signal: controller.signal as any,
        });
        const end = performance.now();
        const ms = Math.max(0, Math.round(end - start));

        // Try to parse JSON (health endpoint should return JSON)
        let body: any = null;
        try {
          body = await res.clone().json();
        } catch {
          // non-JSON is fine
        }

        if (cancelled) return;

        if (res.ok) {
          setLatency(ms);
          setStatus('connected');
          return;
        }

        // 503 + placeholder flag → explicit "no_camera"
        if (res.status === 503 && body && body.placeholder === true) {
          setLatency(ms);
          setStatus('no_camera');
          return;
        }

        // Generic 503 → treat as "connecting" (server reachable but not ready)
        if (res.status === 503 && treat503AsConnecting) {
          setLatency(ms);
          setStatus('connecting');
          return;
        }

        // Anything else → disconnected
        setLatency(null);
        setStatus('disconnected');
      } catch {
        if (!cancelled) {
          setLatency(null);
          setStatus('disconnected'); // network/CORS/timeout
        }
      } finally {
        clearTimeout(timeout);
      }
    };

    // Kick off immediately, then on interval
    probe();
    timerRef.current = setInterval(probe, Math.max(1000, intervalMs));

    return () => {
      cancelled = true;
      if (timerRef.current) clearInterval(timerRef.current);
      timerRef.current = null;
    };
  }, [enabled, url, intervalMs, timeoutMs, treat503AsConnecting]);

  return { status, latency };
}
