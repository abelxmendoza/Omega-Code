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
#   Circuit breaker:
#     After FAILURE_THRESHOLD consecutive non-200 responses, polling backs off to
#     BACKOFF_INTERVAL_MS (60s). Resets to normalInterval on next success.
#     This prevents log spam and 429s when an endpoint is persistently unavailable.
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

const FAILURE_THRESHOLD  = 3;       // consecutive failures before backing off
const BACKOFF_INTERVAL_MS = 60_000; // back-off polling interval after threshold

export function useHttpStatus(url: string | undefined, opts: Options = {}): HttpState {
  const {
    enabled = true,
    intervalMs = 5000,
    timeoutMs = 2500,
    treat503AsConnecting = true,
  } = opts;

  const [status, setStatus] = useState<HttpStatus>('disconnected');
  const [latency, setLatency] = useState<number | null>(null);
  const timerRef        = useRef<ReturnType<typeof setInterval> | null>(null);
  const failureCountRef = useRef(0);
  // Track the currently active interval duration so we can avoid unnecessary restarts
  const activeIntervalRef = useRef(intervalMs);

  useEffect(() => {
    if (timerRef.current) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }

    if (!enabled || !url) {
      setStatus('disconnected');
      setLatency(null);
      failureCountRef.current = 0;
      return;
    }

    // Reset circuit breaker whenever url/enabled/interval changes
    failureCountRef.current = 0;
    activeIntervalRef.current = intervalMs;

    let cancelled = false;

    /** Restart the polling timer at a new interval (circuit breaker escalation/recovery). */
    const restartTimer = (ms: number) => {
      if (timerRef.current) clearInterval(timerRef.current);
      activeIntervalRef.current = ms;
      timerRef.current = setInterval(probe, Math.max(1000, ms));
    };

    const probe = async () => {
      setStatus((s) => (s === 'connected' || s === 'no_camera' ? s : 'connecting'));

      const start = performance.now();
      const controller = new AbortController();
      const timeout = setTimeout(() => controller.abort(), timeoutMs);

      try {
        const res = await fetch(url, {
          method: 'GET',
          cache: 'no-store',
          mode: 'cors' as RequestMode,
          signal: controller.signal as any,
        });
        const end = performance.now();
        const ms = Math.max(0, Math.round(end - start));

        let body: any = null;
        try { body = await res.clone().json(); } catch { /* non-JSON is fine */ }

        if (cancelled) return;

        if (res.ok) {
          // Success: reset circuit breaker and restore normal interval
          const wasBackedOff = failureCountRef.current >= FAILURE_THRESHOLD;
          failureCountRef.current = 0;
          setLatency(ms);
          setStatus('connected');
          if (wasBackedOff) restartTimer(intervalMs); // recover from back-off
          return;
        }

        // 503 + placeholder → explicit no_camera (not a failure — server is intentional)
        if (res.status === 503 && body?.placeholder === true) {
          failureCountRef.current = 0;
          setLatency(ms);
          setStatus('no_camera');
          return;
        }

        // Generic 503 → "connecting" but still counts as a failure for circuit breaker
        if (res.status === 503 && treat503AsConnecting) {
          setLatency(ms);
          setStatus('connecting');
        } else {
          setLatency(null);
          setStatus('disconnected');
        }

        // Circuit breaker: escalate to back-off after repeated failures
        failureCountRef.current += 1;
        if (
          failureCountRef.current === FAILURE_THRESHOLD &&
          activeIntervalRef.current !== BACKOFF_INTERVAL_MS
        ) {
          restartTimer(BACKOFF_INTERVAL_MS);
        }
      } catch {
        if (!cancelled) {
          setLatency(null);
          setStatus('disconnected');
          failureCountRef.current += 1;
          if (
            failureCountRef.current === FAILURE_THRESHOLD &&
            activeIntervalRef.current !== BACKOFF_INTERVAL_MS
          ) {
            restartTimer(BACKOFF_INTERVAL_MS);
          }
        }
      } finally {
        clearTimeout(timeout);
      }
    };

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
