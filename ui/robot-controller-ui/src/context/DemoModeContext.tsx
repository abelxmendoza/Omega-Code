'use client';

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

export type ConnectionMode = 'live' | 'sim-backend' | 'frontend-demo';

interface DemoModeContextType {
  connectionMode:    ConnectionMode;
  setConnectionMode: (mode: ConnectionMode) => void;
  /** true when connectionMode === 'frontend-demo' */
  demoMode:          boolean;
  /** true when connectionMode === 'sim-backend' */
  simBackendMode:    boolean;
  /** Backward-compat shim: true → frontend-demo, false → live */
  setDemoMode:       (enabled: boolean) => void;
  engine:            SimEngine;
  /** false until localStorage has been read — hooks suppress WS while false */
  isHydrated:        boolean;
}

// ── Context ───────────────────────────────────────────────────────────────────

const DemoModeContext = createContext<DemoModeContextType | null>(null);

// ── Provider ──────────────────────────────────────────────────────────────────

const STORAGE_KEY = 'omega_connection_mode';
const VALID_MODES: ConnectionMode[] = ['live', 'sim-backend', 'frontend-demo'];

export function DemoModeProvider({ children }: { children: React.ReactNode }) {
  const [connectionMode, setConnectionModeState] = useState<ConnectionMode>('live');
  const [isHydrated, setIsHydrated] = useState(false);
  const engineRef = useRef<SimEngine>(new SimEngine());

  // Read localStorage after mount (SSR-safe). Sets isHydrated=true so child
  // hooks know it's safe to open WS connections with the correct mode.
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const stored = localStorage.getItem(STORAGE_KEY) as ConnectionMode | null;
      if (stored && VALID_MODES.includes(stored)) {
        setConnectionModeState(stored);
      } else {
        // Migrate legacy 'omega_demo_mode' key
        if (localStorage.getItem('omega_demo_mode') === '1') {
          setConnectionModeState('frontend-demo');
        }
      }
    }
    setIsHydrated(true);
  }, []);

  const setConnectionMode = useCallback((mode: ConnectionMode) => {
    if (typeof window !== 'undefined') {
      localStorage.setItem(STORAGE_KEY, mode);
    }
    setConnectionModeState(mode);
  }, []);

  // Backward-compat shim for existing consumers
  const setDemoMode = useCallback((enabled: boolean) => {
    setConnectionMode(enabled ? 'frontend-demo' : 'live');
  }, [setConnectionMode]);

  const demoMode       = connectionMode === 'frontend-demo';
  const simBackendMode = connectionMode === 'sim-backend';

  // Start / stop SimEngine when entering / leaving frontend-demo mode
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
    () => ({
      connectionMode,
      setConnectionMode,
      demoMode,
      simBackendMode,
      setDemoMode,
      engine: engineRef.current,
      isHydrated,
    }),
    [connectionMode, setConnectionMode, demoMode, simBackendMode, setDemoMode, isHydrated],
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
