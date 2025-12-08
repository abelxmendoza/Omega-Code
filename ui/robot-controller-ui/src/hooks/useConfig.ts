/**
 * useConfig Hook
 * 
 * Fetches and manages full robot configuration.
 */

import { useState, useEffect, useCallback } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

export interface RobotConfig {
  robot: {
    name: string;
    profile: string;
    version: string;
    serial_number?: string;
  };
  network: {
    default_mode: string;
    ap?: {
      ssid: string;
      password: string;
      ip: string;
      dhcp_start: string;
      dhcp_end: string;
    };
    client?: {
      auto_connect: boolean;
      preferred_ssid?: string;
      static_ip?: string | null;
    };
    tailscale?: {
      enabled: boolean;
      tailnet?: string;
    };
  };
  services: {
    autostart: string[];
    restart_policies: Record<string, string>;
  };
  camera: {
    backend: string;
    device: string;
    width: number;
    height: number;
    fps: number;
    test_mode?: boolean;
  };
  movement: {
    default_profile: string;
    max_speed: number;
    min_speed: number;
    servo_center_on_startup?: boolean;
  };
  lighting: {
    default_pattern: string;
    default_brightness: number;
    led_count?: number;
    gpio_pin?: number;
  };
  logging: {
    level: string;
    directory?: string;
    rotation?: {
      enabled: boolean;
      max_size_mb: number;
      backup_count: number;
    };
  };
  security: {
    api_auth_enabled: boolean;
    api_key?: string;
    allowed_origins: string[];
  };
  telemetry: {
    enabled: boolean;
    update_interval_ms: number;
    export_prometheus?: boolean;
    prometheus_port?: number;
  };
}

interface UseConfigReturn {
  config: RobotConfig | null;
  loading: boolean;
  error: Error | null;
  refresh: () => Promise<void>;
}

export function useConfig(): UseConfigReturn {
  const [config, setConfig] = useState<RobotConfig | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const fetchConfig = useCallback(async () => {
    if (!ROBOT_ENABLED) {
      setConfig(null);
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch('/api/config');
      
      if (response.offline) {
        setConfig(null);
        setError(null);
        return;
      }

      const data = await response.json();
      
      if (data.ok && data.config) {
        setConfig(data.config as RobotConfig);
        setError(null);
      } else {
        throw new Error('Invalid response format');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to fetch config');
      setError(error);
      console.error('Failed to fetch config:', error);
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchConfig();
  }, [fetchConfig]);

  return {
    config,
    loading,
    error,
    refresh: fetchConfig,
  };
}

