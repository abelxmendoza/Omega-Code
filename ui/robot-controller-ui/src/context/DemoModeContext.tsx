'use client';

/**
 * DemoModeContext — frontend-only simulation mode.
 *
 * Provides:
 *   demoMode    — boolean, read by any hook/component that needs to branch
 *   setDemoMode — toggle
 *   engine      — shared SimEngine instance (started/stopped with demoMode)
 *
 * Must be placed above CommandProvider and usePoseStream consumers in the
 * provider tree so both can call useDemoMode() safely.
 */

import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import { SimEngine } from '@/utils/sim_engine';

// ── Types ─────────────────────────────────────────────────────────────────────

interface DemoModeContextType {
  demoMode:    boolean;
  setDemoMode: (enabled: boolean) => void;
  engine:      SimEngine;
  /** false until localStorage has been read — suppress all WS connections while false */
  isHydrated:  boolean;
}

// ── Context ───────────────────────────────────────────────────────────────────

const DemoModeContext = createContext<DemoModeContextType | null>(null);

// ── Provider ──────────────────────────────────────────────────────────────────

const STORAGE_KEY = 'omega_demo_mode';

export function DemoModeProvider({ children }: { children: React.ReactNode }) {
  const [demoMode, setDemoModeState] = useState(false);
  const [isHydrated, setIsHydrated] = useState(false);
  const engineRef = useRef<SimEngine>(new SimEngine());

  // Read localStorage after mount (SSR-safe). Sets isHydrated=true so child
  // hooks know it's safe to open WS connections using the correct demoMode.
  useEffect(() => {
    if (typeof window !== 'undefined' && localStorage.getItem(STORAGE_KEY) === '1') {
      setDemoModeState(true);
    }
    setIsHydrated(true);
  }, []);

  const setDemoMode = useCallback((enabled: boolean) => {
    if (typeof window !== 'undefined') {
      localStorage.setItem(STORAGE_KEY, enabled ? '1' : '0');
    }
    setDemoModeState(enabled);
  }, []);

  // Start / stop engine with mode
  useEffect(() => {
    const engine = engineRef.current;
    if (demoMode) {
      engine.start();
    } else {
      engine.stop();
      engine.reset();
    }
  }, [demoMode]);

  // Always stop on unmount
  useEffect(() => {
    const engine = engineRef.current;
    return () => engine.stop();
  }, []);

  const value = useMemo<DemoModeContextType>(
    () => ({ demoMode, setDemoMode, engine: engineRef.current, isHydrated }),
    [demoMode, setDemoMode, isHydrated],
  );

  return (
    <DemoModeContext.Provider value={value}>
      {children}
    </DemoModeContext.Provider>
  );
}

// ── Hook ──────────────────────────────────────────────────────────────────────

export function useDemoMode(): DemoModeContextType {
  const ctx = useContext(DemoModeContext);
  if (!ctx) throw new Error('useDemoMode must be used within DemoModeProvider');
  return ctx;
}
