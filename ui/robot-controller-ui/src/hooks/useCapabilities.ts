import { useState, useEffect } from 'react';
import { robotFetch } from '@/utils/network';

export interface CapabilityProfile {
  device: string;
  hostname: string;
  arch: string;
  os: string;
  os_version: string;
  is_jetson: boolean;
  cuda: boolean;
  ros2_dev: boolean;
  ml_capable: boolean;
  slam_capable: boolean;
  tracking: boolean;
  aruco: boolean;
  motion_detection: boolean;
  face_recognition: boolean;
  yolo: boolean;
  max_resolution: string;
  max_fps: number;
  profile_mode: 'mac' | 'lenovo' | 'jetson' | 'unknown';
  gpu_available: boolean;
  gpu_name: string | null;
  cpu_count: number;
}

export interface CapabilitiesResponse {
  ok: boolean;
  capabilities?: CapabilityProfile;
  error?: string;
}

export function useCapabilities(autoRefresh = true, refreshInterval = 30000, enabled = true) {
  const [capabilities, setCapabilities] = useState<CapabilityProfile | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const fetchCapabilities = async (forceRefresh = false) => {
    try {
      setLoading(true);
      setError(null);

      const path = `/api/capabilities/${forceRefresh ? '?refresh=true' : ''}`;
      // Use robotFetch wrapper to respect offline mode
      const response = await robotFetch(path);
      
      if ((response as any).offline) {
        setError('Robot backend offline');
        setLoading(false);
        return;
      }
      const data: CapabilitiesResponse = await response.json();
      
      if (data.ok && data.capabilities) {
        setCapabilities(data.capabilities);
      } else {
        setError(data.error || 'Failed to fetch capabilities');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
      console.error('Failed to fetch capabilities:', err);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    if (!enabled) return;
    fetchCapabilities();

    if (autoRefresh) {
      const interval = setInterval(() => {
        fetchCapabilities();
      }, refreshInterval);

      return () => clearInterval(interval);
    }
  }, [autoRefresh, refreshInterval, enabled]);

  return {
    capabilities,
    loading,
    error,
    refresh: () => fetchCapabilities(true),
    isMLCapable: capabilities?.ml_capable ?? false,
    isSLAMCapable: capabilities?.slam_capable ?? false,
    isJetsonMode: capabilities?.profile_mode === 'jetson',
    isDevMode: capabilities?.profile_mode === 'lenovo',
    isLightMode: capabilities?.profile_mode === 'mac',
    maxResolution: capabilities?.max_resolution ?? '640x480',
    maxFPS: capabilities?.max_fps ?? 30,
    profileMode: capabilities?.profile_mode ?? 'unknown',
  };
}

const API_BASE_CAP =
  (process.env.NEXT_PUBLIC_API_URL ?? '').replace(/\/$/, '') || 'http://localhost:8000';

export function useCapabilityCheck(feature: string) {
  const [available, setAvailable] = useState<boolean | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const checkCapability = async () => {
      try {
        setLoading(true);
        const response = await fetch(`${API_BASE_CAP}/api/capabilities/check?feature=${feature}`);
        const data = await response.json();
        
        if (data.ok) {
          setAvailable(data.available ?? false);
        }
      } catch (err) {
        console.error(`Failed to check capability ${feature}:`, err);
        setAvailable(false);
      } finally {
        setLoading(false);
      }
    };

    checkCapability();
  }, [feature]);

  return { available, loading };
}

