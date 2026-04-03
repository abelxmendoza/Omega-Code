/**
 * useServiceSafety
 *
 * Lightweight hook that derives system-readiness signals from the service list.
 * Polls /api/services/list every 10s with an in-flight guard.
 *
 * Critical service map:
 *   movement   → movement_ws_server
 *   sensors    → ultrasonic_ws_server + line_tracking_ws_server
 *   camera     → video_server
 *
 * Exposes:
 *   systemReady    — all three critical groups running
 *   movementReady  — movement_ws_server is running
 *   sensorsReady   — at least one sensor WS server is running
 *   cameraReady    — video_server is running
 *   readinessLevel — 'ready' | 'degraded' | 'critical'
 *   blockedReason  — human-readable string when autonomy should be blocked
 */

import { useState, useEffect, useRef } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

export type ReadinessLevel = 'ready' | 'degraded' | 'critical';

export interface ServiceSafety {
  systemReady:   boolean;
  movementReady: boolean;
  sensorsReady:  boolean;
  cameraReady:   boolean;
  readinessLevel: ReadinessLevel;
  /** Non-null when autonomy-start should be blocked */
  blockedReason: string | null;
}

const MOVEMENT_SERVICES = ['movement_ws_server'];
const SENSOR_SERVICES   = ['ultrasonic_ws_server', 'line_tracking_ws_server'];
const CAMERA_SERVICES   = ['video_server'];

const POLL_MS    = 10_000;
const BACKOFF_MS = 60_000;
const FAIL_THRESH = 3;

const SAFE_DEFAULT: ServiceSafety = {
  systemReady:    false,
  movementReady:  false,
  sensorsReady:   false,
  cameraReady:    false,
  readinessLevel: 'critical',
  blockedReason:  null,
};

function deriveReady(services: { name: string; status: string }[]): ServiceSafety {
  const isRunning = (names: string[]) =>
    names.some(n => services.find(s => s.name === n)?.status === 'running');

  const movementReady = isRunning(MOVEMENT_SERVICES);
  const sensorsReady  = isRunning(SENSOR_SERVICES);
  const cameraReady   = isRunning(CAMERA_SERVICES);
  const systemReady   = movementReady && sensorsReady && cameraReady;

  const readinessLevel: ReadinessLevel =
    !movementReady || !sensorsReady ? 'critical' :
    !cameraReady                    ? 'degraded' :
    'ready';

  const reasons: string[] = [];
  if (!movementReady) reasons.push('Movement service offline');
  if (!sensorsReady)  reasons.push('Sensor service offline');

  const blockedReason = reasons.length > 0 ? reasons.join(' · ') : null;

  return { systemReady, movementReady, sensorsReady, cameraReady, readinessLevel, blockedReason };
}

export function useServiceSafety(): ServiceSafety {
  const [safety, setSafety] = useState<ServiceSafety>(SAFE_DEFAULT);
  const isFetchingRef = useRef(false);
  const mountedRef    = useRef(true);
  const fetchRef      = useRef<() => void>(() => {});

  useEffect(() => {
    mountedRef.current = true;
    return () => { mountedRef.current = false; };
  }, []);

  useEffect(() => {
    if (!ROBOT_ENABLED) return;

    let failCount  = 0;
    let intervalId: ReturnType<typeof setInterval>;

    async function fetchSafety() {
      if (isFetchingRef.current) return;
      isFetchingRef.current = true;
      try {
        const res = await robotFetch('/api/services/list');
        if (!mountedRef.current) return;

        if ('offline' in res && res.offline) { failCount++; return; }
        if (!res.ok) { failCount++; return; }

        const data = await res.json();
        if (!mountedRef.current) return;

        if (data.ok && Array.isArray(data.services)) {
          failCount = 0;
          setSafety(deriveReady(data.services));
          if (failCount === 0) {
            clearInterval(intervalId);
            intervalId = setInterval(fetchSafety, POLL_MS);
          }
        } else {
          failCount++;
        }
      } catch {
        if (mountedRef.current) failCount++;
      } finally {
        isFetchingRef.current = false;
      }

      if (failCount >= FAIL_THRESH) {
        clearInterval(intervalId);
        intervalId = setInterval(fetchSafety, BACKOFF_MS);
      }
    }

    fetchRef.current = fetchSafety;
    fetchSafety();
    intervalId = setInterval(fetchSafety, POLL_MS);

    return () => clearInterval(intervalId);
  }, []);

  return safety;
}
