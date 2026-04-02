'use client';

/**
 * useSystemMode — single shared polling source for vision mode state.
 *
 * Both VisionModePanel (control) and SystemStatusPanel (read-only diagnostics)
 * consume this hook instead of maintaining independent poll loops.
 *
 * Poll interval: 15 s.
 * Circuit breaker: 3 consecutive failures → 60 s back-off.
 * On success after back-off: automatically resets to 15 s on the next mount.
 *
 * Returns a `refresh()` function so the control panel can trigger an
 * immediate re-fetch right after a mode-set POST, without waiting 15 s.
 */

import { useState, useEffect, useRef, useCallback } from 'react';

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

export interface SystemModeState {
  /** Current mode integer (null = not yet fetched or robot offline) */
  mode: number | null;
  /** Backend-authoritative description string */
  description: string;
  /** Backend-authoritative mode name (e.g. "MOTION_DETECTION") */
  modeName: string;
  /** True when mode was set manually via the UI (vs auto-detected) */
  manualOverride: boolean;
  /** True when Jetson Orin is available (gates YOLO / Autonomy buttons) */
  orinAvailable: boolean;
  /** Hybrid system mode name, or null when Pi-only */
  hybridMode: string | null;
  /** Pi CPU temperature °C (0 when hybrid system not running) */
  thermalTemp: number;
  /** Pi CPU load % (0 when not available) */
  cpuLoad: number;
  /** True when hybrid thermal/CPU throttle is active */
  throttling: boolean;
  /** True when the backend has not responded in FAIL_THRESHOLD polls */
  offline: boolean;
}

const INITIAL: SystemModeState = {
  mode: null,
  description: '',
  modeName: '',
  manualOverride: false,
  orinAvailable: false,
  hybridMode: null,
  thermalTemp: 0,
  cpuLoad: 0,
  throttling: false,
  offline: false,
};

const POLL_MS       = 15_000;
const BACKOFF_MS    = 60_000;
const FAIL_THRESHOLD = 3;

/* ------------------------------------------------------------------ */
/* Hook                                                                */
/* ------------------------------------------------------------------ */

export function useSystemMode(): SystemModeState & { refresh: () => void } {
  const [state, setState] = useState<SystemModeState>(INITIAL);

  // Stable ref so refresh() doesn't need to be recreated on every render
  const fetchRef = useRef<() => void>(() => {});

  useEffect(() => {
    let mounted = true;
    let failCount = 0;
    let intervalId: ReturnType<typeof setInterval>;

    async function fetchStatus() {
      try {
        const res = await fetch('/api/system/mode/status');
        if (!mounted) return;

        if (res.ok) {
          failCount = 0;
          const d = await res.json();
          setState({
            mode:          d.mode          ?? null,
            description:   d.description   ?? '',
            modeName:      d.mode_name     ?? '',
            manualOverride:!!d.manual_override,
            orinAvailable: !!d.orin_available,
            hybridMode:    d.hybrid_mode   ?? null,
            thermalTemp:   d.thermal_temp  ?? 0,
            cpuLoad:       d.cpu_load      ?? 0,
            throttling:    !!d.throttling,
            offline:       false,
          });
        } else {
          failCount++;
          if (failCount >= FAIL_THRESHOLD) {
            setState(s => ({ ...s, offline: true }));
            clearInterval(intervalId);
            intervalId = setInterval(fetchStatus, BACKOFF_MS);
          }
        }
      } catch {
        if (!mounted) return;
        failCount++;
        if (failCount >= FAIL_THRESHOLD) {
          setState(s => ({ ...s, offline: true }));
          clearInterval(intervalId);
          intervalId = setInterval(fetchStatus, BACKOFF_MS);
        }
      }
    }

    // Expose fetchStatus so refresh() can call it without re-running the effect
    fetchRef.current = fetchStatus;

    fetchStatus();
    intervalId = setInterval(fetchStatus, POLL_MS);

    return () => {
      mounted = false;
      clearInterval(intervalId);
    };
  }, []);

  const refresh = useCallback(() => { fetchRef.current(); }, []);

  return { ...state, refresh };
}
