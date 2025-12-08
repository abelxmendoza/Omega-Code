/**
 * useServiceStatus Hook
 * 
 * Shared hook for polling service status from the API.
 * Provides real-time service status updates.
 */

import { useState, useEffect, useCallback } from 'react';
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
  /** Polling interval in milliseconds (default: 5000) */
  interval?: number;
  /** Whether to start polling immediately (default: true) */
  autoStart?: boolean;
}

interface UseServiceStatusReturn {
  services: ServiceStatus[];
  loading: boolean;
  error: Error | null;
  refresh: () => Promise<void>;
  startPolling: () => void;
  stopPolling: () => void;
}

export function useServiceStatus(
  options: UseServiceStatusOptions = {}
): UseServiceStatusReturn {
  const { interval = 5000, autoStart = true } = options;

  const [services, setServices] = useState<ServiceStatus[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);
  const [polling, setPolling] = useState(autoStart);

  const fetchServices = useCallback(async () => {
    if (!ROBOT_ENABLED) {
      setServices([]);
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch('/api/services/list');
      
      if (response.offline) {
        setServices([]);
        setError(null);
        return;
      }

      const data = await response.json();
      
      if (data.ok && Array.isArray(data.services)) {
        setServices(data.services);
        setError(null);
      } else {
        throw new Error('Invalid response format');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to fetch services');
      setError(error);
      console.error('Failed to fetch services:', error);
    } finally {
      setLoading(false);
    }
  }, []);

  // Polling effect
  useEffect(() => {
    if (!polling) return;

    // Initial fetch
    fetchServices();

    // Set up interval
    const intervalId = setInterval(fetchServices, interval);

    return () => clearInterval(intervalId);
  }, [polling, interval, fetchServices]);

  const startPolling = useCallback(() => {
    setPolling(true);
  }, []);

  const stopPolling = useCallback(() => {
    setPolling(false);
  }, []);

  return {
    services,
    loading,
    error,
    refresh: fetchServices,
    startPolling,
    stopPolling,
  };
}

