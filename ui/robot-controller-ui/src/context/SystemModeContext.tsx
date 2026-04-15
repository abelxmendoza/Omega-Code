'use client';

/**
 * SystemModeContext — single polling loop shared by all consumers.
 *
 * Problem it solves: useSystemMode() is called by both SystemStatusPanel and
 * VisionModePanel.  Without a context each call creates its own polling loop,
 * doubling the requests to /api/system/mode/status.  This context wraps the
 * existing hook logic into a provider so both components share one loop.
 *
 * Usage:
 *   1. Wrap the app (or the relevant subtree) in <SystemModeProvider>.
 *   2. Replace `useSystemMode()` calls with `useSystemModeContext()`.
 *
 * The hook logic is unchanged from hooks/useSystemMode.ts — only the
 * instantiation point has moved so it runs exactly once.
 */

import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  useRef,
  useCallback,
} from 'react';

import type { SystemModeState } from '@/hooks/useSystemMode';

// ── Context types ──────────────────────────────────────────────────────────────

type SystemModeContextValue = SystemModeState & { refresh: () => void };

const SystemModeContext = createContext<SystemModeContextValue | null>(null);

// ── Constants (same as hooks/useSystemMode.ts) ─────────────────────────────────

const POLL_MS        = 15_000;
const BACKOFF_MS     = 60_000;
const FAIL_THRESHOLD = 3;

const INITIAL: SystemModeState = {
  mode:           null,
  description:    '',
  modeName:       '',
  manualOverride: false,
  orinAvailable:  false,
  hybridMode:     null,
  thermalTemp:    0,
  cpuLoad:        0,
  throttling:     false,
  offline:        false,
};

// ── Provider ───────────────────────────────────────────────────────────────────

export function SystemModeProvider({ children }: { children: React.ReactNode }) {
  const [state, setState] = useState<SystemModeState>(INITIAL);
  const fetchRef          = useRef<() => void>(() => {});

  useEffect(() => {
    let mounted   = true;
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
            mode:           d.mode          ?? null,
            description:    d.description   ?? '',
            modeName:       d.mode_name     ?? '',
            manualOverride: !!d.manual_override,
            orinAvailable:  !!d.orin_available,
            hybridMode:     d.hybrid_mode   ?? null,
            thermalTemp:    d.thermal_temp  ?? 0,
            cpuLoad:        d.cpu_load      ?? 0,
            throttling:     !!d.throttling,
            offline:        false,
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

    fetchRef.current = fetchStatus;
    fetchStatus();
    intervalId = setInterval(fetchStatus, POLL_MS);

    return () => {
      mounted = false;
      clearInterval(intervalId);
    };
  }, []);

  const refresh = useCallback(() => { fetchRef.current(); }, []);

  return (
    <SystemModeContext.Provider value={{ ...state, refresh }}>
      {children}
    </SystemModeContext.Provider>
  );
}

// ── Consumer hook ──────────────────────────────────────────────────────────────

export function useSystemModeContext(): SystemModeContextValue {
  const ctx = useContext(SystemModeContext);
  if (!ctx) {
    throw new Error('useSystemModeContext must be used inside <SystemModeProvider>');
  }
  return ctx;
}
