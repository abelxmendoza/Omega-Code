/**
 * useServiceStatus
 *
 * Single polling source for service status.
 * - In-flight guard: skips fetch if one is already running (prevents overlap on slow networks)
 * - Circuit breaker: 3 consecutive failures → 60s backoff
 * - Exposes lastUpdated timestamp and immediate refresh()
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

export interface ServiceStatus {
  name: string;
  display_name: string;
  description: string;
  type: string;
  port?: number;
  autostart: boolean;
  status: string;
  pid?: number;
  restart_policy: string;
  crash_count: number;
  cpu_percent?: number;
  memory_mb?: number;
  health?: {
    healthy: boolean;
    message: string;
  };
}

interface UseServiceStatusOptions {
  /** Polling interval in ms (default: 5000) */
  interval?: number;
}

interface UseServiceStatusReturn {
  services: ServiceStatus[];
  loading: boolean;
  error: Error | null;
  lastUpdated: number;       // epoch ms of last successful fetch (0 = never)
  refresh: () => void;       // triggers an immediate fetch (no-op if one is in flight)
}

const BACKOFF_MS      = 60_000;
const FAIL_THRESHOLD  = 3;

export function useServiceStatus(
  options: UseServiceStatusOptions = {}
): UseServiceStatusReturn {
  const { interval = 5000 } = options;

  const [services,    setServices]    = useState<ServiceStatus[]>([]);
  const [loading,     setLoading]     = useState(false);
  const [error,       setError]       = useState<Error | null>(null);
  const [lastUpdated, setLastUpdated] = useState(0);

  // Stable refs — never trigger re-renders, never become stale in closures
  const isFetchingRef = useRef(false);
  const fetchFnRef    = useRef<() => void>(() => {});
  const mountedRef    = useRef(true);

  useEffect(() => {
    mountedRef.current = true;
    return () => { mountedRef.current = false; };
  }, []);

  useEffect(() => {
    if (!ROBOT_ENABLED) {
      setServices([]);
      setError(null);
      return;
    }

    let failCount  = 0;
    let intervalId: ReturnType<typeof setInterval>;

    async function fetchServices() {
      // In-flight guard — skip if a fetch is already running
      if (isFetchingRef.current) return;
      isFetchingRef.current = true;
      setLoading(true);

      try {
        const res = await robotFetch('/api/services/list');
        if (!mountedRef.current) return;

        if ('offline' in res && res.offline) {
          // Treat offline the same as a fetch failure
          failCount++;
        } else if (res.ok) {
          const data = await res.json();
          if (!mountedRef.current) return;

          if (data.ok && Array.isArray(data.services)) {
            failCount = 0;
            setServices(data.services);
            setLastUpdated(Date.now());
            setError(null);

            // Recover from backoff if we were in it
            if (failCount === 0) {
              clearInterval(intervalId);
              intervalId = setInterval(fetchServices, interval);
            }
          } else {
            failCount++;
            setError(new Error('Invalid response format'));
          }
        } else {
          failCount++;
          setError(new Error(`HTTP ${res.status}`));
        }
      } catch (err) {
        if (!mountedRef.current) return;
        failCount++;
        setError(err instanceof Error ? err : new Error('Failed to fetch services'));
      } finally {
        if (mountedRef.current) setLoading(false);
        isFetchingRef.current = false;
      }

      // Circuit breaker: escalate to backoff after threshold
      if (failCount >= FAIL_THRESHOLD) {
        clearInterval(intervalId);
        intervalId = setInterval(fetchServices, BACKOFF_MS);
      }
    }

    fetchFnRef.current = fetchServices;

    fetchServices();
    intervalId = setInterval(fetchServices, interval);

    return () => clearInterval(intervalId);
  }, [interval]);

  // Stable refresh — triggers an immediate fetch regardless of interval
  const refresh = useCallback(() => {
    fetchFnRef.current();
  }, []);

  return { services, loading, error, lastUpdated, refresh };
}
