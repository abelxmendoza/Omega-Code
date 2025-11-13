/**
 * Hook to check ROS2 status and availability
 * 
 * Checks if ROS2 is available and what topics/actions are active
 */

import { useState, useEffect } from 'react';
import { buildGatewayUrl } from '@/utils/gateway';

export interface ROS2Status {
  available: boolean;
  connected: boolean;
  mode: 'native' | 'docker' | 'none';
  topics: string[];
  actions: string[];
  error?: string;
}

export function useROS2Status(autoRefresh = true, refreshInterval = 5000) {
  const [status, setStatus] = useState<ROS2Status>({
    available: false,
    connected: false,
    mode: 'none',
    topics: [],
    actions: [],
  });

  const checkStatus = async () => {
    try {
      const url = await buildGatewayUrl('/api/ros/status');
      const response = await fetch(url);
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      
      const data = await response.json();
      
      setStatus({
        available: data.mode !== 'none',
        connected: data.mode !== 'none',
        mode: data.mode || 'none',
        topics: data.topics || [],
        actions: [], // Would need separate endpoint for actions
        error: undefined,
      });
    } catch (err) {
      setStatus(prev => ({
        ...prev,
        available: false,
        connected: false,
        error: err instanceof Error ? err.message : 'Unknown error',
      }));
    }
  };

  useEffect(() => {
    if (autoRefresh) {
      checkStatus();
      const interval = setInterval(checkStatus, refreshInterval);
      return () => clearInterval(interval);
    }
  }, [autoRefresh, refreshInterval]);

  return { status, refresh: checkStatus };
}

