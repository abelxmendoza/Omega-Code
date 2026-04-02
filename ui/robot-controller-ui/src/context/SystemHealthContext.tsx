/*
# File: /src/context/SystemHealthContext.tsx
# Summary:
#   Global provider for the singleton systemHealth service.
#   Mount <SystemHealthProvider> once at the app root — all components can then
#   call useSystemHealth() to read the centralized health state without running
#   their own polling loops.
#
#   This solves the N×polling problem: instead of every component independently
#   probing /api/health, there is exactly ONE probe loop (in systemHealth.ts)
#   and components subscribe via React context.
#
#   Usage:
#     // _app.tsx or index.tsx wrapper
#     <SystemHealthProvider>
#       <App />
#     </SystemHealthProvider>
#
#     // Any component
#     const health = useSystemHealth();
#     if (!health.connected) return <Offline />;
*/

'use client';

import React, { createContext, useContext, useEffect, useState } from 'react';
import { systemHealth, SystemHealthState } from '@/services/systemHealth';

interface SystemHealthContextValue {
  health: SystemHealthState;
}

const SystemHealthContext = createContext<SystemHealthContextValue | null>(null);

export function SystemHealthProvider({ children }: { children: React.ReactNode }) {
  const [health, setHealth] = useState<SystemHealthState>(() => systemHealth.getState());

  useEffect(() => {
    // Start the singleton polling loop (no-op if already running)
    systemHealth.start();
    // Subscribe: receive every state update from the single polling loop
    const unsub = systemHealth.subscribe(setHealth);
    return () => {
      unsub();
      systemHealth.stop();
    };
  }, []);

  return (
    <SystemHealthContext.Provider value={{ health }}>
      {children}
    </SystemHealthContext.Provider>
  );
}

/**
 * Returns the current system health state from the centralized polling service.
 * Must be called inside a <SystemHealthProvider>.
 *
 * @example
 *   const health = useSystemHealth();
 *   health.connected          // boolean — backend reachable
 *   health.degraded           // boolean — 3+ consecutive failures
 *   health.components.backend // 'ok' | 'degraded' | 'down'
 */
export function useSystemHealth(): SystemHealthState {
  const ctx = useContext(SystemHealthContext);
  if (!ctx) {
    throw new Error('useSystemHealth must be used within a <SystemHealthProvider>');
  }
  return ctx.health;
}
