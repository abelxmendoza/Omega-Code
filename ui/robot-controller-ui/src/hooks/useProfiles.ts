/**
 * useProfiles Hook
 * 
 * Fetches robot profile information.
 */

import { useState, useEffect, useCallback } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

export interface RobotProfile {
  name: string;
  description: string;
  hardware: {
    cpu_cores: number;
    ram_gb: number;
    gpu: string;
    camera: string;
  };
  capabilities: {
    ml_capable: boolean;
    slam_capable: boolean;
    max_resolution: string;
    max_fps: number;
    gpu_acceleration: boolean;
  };
  recommended_settings: {
    camera_backend: string;
    camera_width: number;
    camera_height: number;
    camera_fps: number;
    movement_profile: string;
  };
}

interface UseProfilesReturn {
  profiles: Record<string, RobotProfile>;
  activeProfile: string | null;
  loading: boolean;
  error: Error | null;
  getProfile: (name: string) => Promise<RobotProfile | null>;
  refresh: () => Promise<void>;
}

export function useProfiles(): UseProfilesReturn {
  const [profiles, setProfiles] = useState<Record<string, RobotProfile>>({});
  const [activeProfile, setActiveProfile] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const fetchProfile = useCallback(async (name: string): Promise<RobotProfile | null> => {
    if (!ROBOT_ENABLED) {
      return null;
    }

    try {
      const response = await robotFetch(`/api/config/profile/${name}`);
      
      if (response.offline) {
        return null;
      }

      const data = await response.json();
      
      if (data.ok && data.data) {
        return data.data as RobotProfile;
      }
      return null;
    } catch (err) {
      console.error(`Failed to fetch profile ${name}:`, err);
      return null;
    }
  }, []);

  const loadAllProfiles = useCallback(async () => {
    if (!ROBOT_ENABLED) {
      setProfiles({});
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Load common profiles
      const profileNames = ['pi4b', 'jetson', 'dev'];
      const profilePromises = profileNames.map(name => 
        fetchProfile(name).then(profile => ({ name, profile }))
      );
      
      const results = await Promise.all(profilePromises);
      const profilesMap: Record<string, RobotProfile> = {};
      
      results.forEach(({ name, profile }) => {
        if (profile) {
          profilesMap[name] = profile;
        }
      });
      
      setProfiles(profilesMap);
      
      // Get active profile from config
      const configResponse = await robotFetch('/api/config');
      if (!configResponse.offline) {
        const configData = await configResponse.json();
        if (configData.ok && configData.config?.robot?.profile) {
          setActiveProfile(configData.config.robot.profile);
        }
      }
      
      setError(null);
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to load profiles');
      setError(error);
      console.error('Failed to load profiles:', error);
    } finally {
      setLoading(false);
    }
  }, [fetchProfile]);

  useEffect(() => {
    loadAllProfiles();
  }, [loadAllProfiles]);

  return {
    profiles,
    activeProfile,
    loading,
    error,
    getProfile: fetchProfile,
    refresh: loadAllProfiles,
  };
}

