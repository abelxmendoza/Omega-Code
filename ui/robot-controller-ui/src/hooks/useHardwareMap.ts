/**
 * useHardwareMap Hook
 * 
 * Fetches hardware device mapping.
 */

import { useState, useEffect, useCallback } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

export interface HardwareMap {
  gpio?: Record<string, number>;
  i2c?: {
    bus?: number;
    devices?: Record<string, {
      address?: string;
      description?: string;
      enabled?: boolean;
    }>;
  };
  spi?: {
    enabled?: boolean;
    bus?: number;
    devices?: any[];
  };
  uart?: {
    enabled?: boolean;
    devices?: any[];
  };
  camera?: {
    csi?: {
      enabled?: boolean;
      device?: string;
      description?: string;
    };
    usb?: {
      enabled?: boolean;
      device?: string;
      description?: string;
    };
  };
  power?: {
    voltage_monitor?: {
      enabled?: boolean;
      pin?: number | null;
    };
    battery_monitor?: {
      enabled?: boolean;
      i2c_address?: string | null;
    };
  };
  network?: {
    wifi?: {
      interface?: string;
    };
    bluetooth?: {
      interface?: string;
      enabled?: boolean;
    };
  };
  validation?: {
    required_devices?: string[];
    optional_devices?: string[];
  };
}

interface UseHardwareMapReturn {
  hardware: HardwareMap | null;
  loading: boolean;
  error: Error | null;
  refresh: () => Promise<void>;
}

export function useHardwareMap(): UseHardwareMapReturn {
  const [hardware, setHardware] = useState<HardwareMap | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const fetchHardware = useCallback(async () => {
    if (!ROBOT_ENABLED) {
      setHardware(null);
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch('/api/config/hardware/map');
      
      if (response.offline) {
        setHardware(null);
        setError(null);
        return;
      }

      const data = await response.json();
      
      if (data.ok && data.hardware) {
        setHardware(data.hardware as HardwareMap);
        setError(null);
      } else {
        throw new Error('Invalid response format');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to fetch hardware map');
      setError(error);
      console.error('Failed to fetch hardware map:', error);
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchHardware();
  }, [fetchHardware]);

  return {
    hardware,
    loading,
    error,
    refresh: fetchHardware,
  };
}

