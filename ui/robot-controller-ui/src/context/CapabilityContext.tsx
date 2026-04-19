import React, { createContext, useContext, ReactNode } from 'react';
import { useCapabilities, CapabilityProfile } from '@/hooks/useCapabilities';
import { useDemoMode } from '@/context/DemoModeContext';

interface CapabilityContextType {
  capabilities: CapabilityProfile | null;
  loading: boolean;
  error: string | null;
  refresh: () => void;
  isMLCapable: boolean;
  isSLAMCapable: boolean;
  isJetsonMode: boolean;
  isDevMode: boolean;
  isLightMode: boolean;
  maxResolution: string;
  maxFPS: number;
  profileMode: string;
}

const CapabilityContext = createContext<CapabilityContextType | undefined>(undefined);

export function CapabilityProvider({ children }: { children: ReactNode }) {
  const { demoMode, simBackendMode, isHydrated } = useDemoMode();
  // Don't fetch capabilities in demo mode, sim-backend mode, or before localStorage is read
  const capabilitiesData = useCapabilities(true, 30000, isHydrated && !demoMode && !simBackendMode);

  return (
    <CapabilityContext.Provider value={capabilitiesData}>
      {children}
    </CapabilityContext.Provider>
  );
}

export function useCapabilityContext() {
  const context = useContext(CapabilityContext);
  if (context === undefined) {
    throw new Error('useCapabilityContext must be used within a CapabilityProvider');
  }
  return context;
}

// Convenience hooks
export function useIsMLCapable() {
  const { isMLCapable } = useCapabilityContext();
  return isMLCapable;
}

export function useIsSLAMCapable() {
  const { isSLAMCapable } = useCapabilityContext();
  return isSLAMCapable;
}

export function useProfileMode() {
  const { profileMode } = useCapabilityContext();
  return profileMode;
}

export function useMaxResolution() {
  const { maxResolution } = useCapabilityContext();
  return maxResolution;
}

