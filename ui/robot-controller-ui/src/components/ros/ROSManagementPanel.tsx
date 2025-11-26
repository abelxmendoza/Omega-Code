'use client';

/**
 * ROS 2 Management Panel Component
 * 
 * Provides UI for managing ROS 2 Docker containers:
 * - Start/stop containers
 * - View container status
 * - Monitor ROS topics
 * - View telemetry logs
 */

import React, { useState, useEffect, useCallback } from 'react';
import { buildGatewayUrl } from '@/config/gateway';

interface ContainerStatus {
  Name: string;
  State: string;
  Status: string;
}

interface ROSStatus {
  containers: ContainerStatus[];
  topics: string[];
  compose_path?: string;
}

interface ROSManagementPanelProps {
  className?: string;
}

export const ROSManagementPanel: React.FC<ROSManagementPanelProps> = ({ className = '' }) => {
  const [status, setStatus] = useState<ROSStatus | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [actionLoading, setActionLoading] = useState<string | null>(null);
  const [logs, setLogs] = useState<{ [key: string]: string[] }>({});

  const fetchStatus = useCallback(async () => {
    try {
      setLoading(true);
      setError(null);
      const url = await buildGatewayUrl('/api/ros/status');
      const response = await fetch(url);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data = await response.json();
      setStatus(data);
      
      // Show message if ROS is disabled
      if (data.mode === 'disabled' || data.message?.includes('disabled')) {
        setError('ROS features are disabled. Set ROS_ENABLED=true on the backend to enable.');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch ROS status');
      setStatus({ containers: [], topics: [] });
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 5000); // Poll every 5s
    return () => clearInterval(interval);
  }, [fetchStatus]);

  const handleControl = async (action: 'start' | 'stop' | 'restart', service?: string) => {
    try {
      setActionLoading(`${action}-${service || 'all'}`);
      setError(null);
      const url = await buildGatewayUrl('/api/ros/control');
      const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, service }),
      });
      
      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || error.error || `Failed to ${action} container`);
      }
      
      const result = await response.json();
      if (result.success) {
        // Refresh status after a short delay
        setTimeout(() => fetchStatus(), 1000);
      } else {
        throw new Error(result.error || 'Unknown error');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Action failed');
    } finally {
      setActionLoading(null);
    }
  };

  const fetchLogs = async (service: string) => {
    try {
      const url = await buildGatewayUrl(`/api/ros/logs/${service}`);
      const response = await fetch(url);
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data = await response.json();
      setLogs(prev => ({ ...prev, [service]: data.logs || [] }));
    } catch (err) {
      setLogs(prev => ({ ...prev, [service]: [`Error: ${err instanceof Error ? err.message : 'Failed to fetch logs'}`] }));
    }
  };

  const getStatusColor = (state: string) => {
    switch (state.toLowerCase()) {
      case 'running':
        return 'bg-green-500';
      case 'stopped':
      case 'exited':
        return 'bg-red-500';
      default:
        return 'bg-yellow-500';
    }
  };

  if (loading && !status) {
    return (
      <div className={`p-4 bg-gray-900 rounded-lg ${className}`}>
        <div className="text-white">Loading ROS status...</div>
      </div>
    );
  }

  return (
    <div className={`p-4 bg-gray-900 rounded-lg text-white ${className}`}>
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold">ROS 2 Docker Management</h3>
        <button
          onClick={() => fetchStatus()}
          className="px-3 py-1 bg-blue-600 hover:bg-blue-700 rounded text-sm"
          disabled={loading}
        >
          {loading ? 'Refreshing...' : 'Refresh'}
        </button>
      </div>

      {error && (
        (() => {
          console.error('ROSManagementPanel Error:', error);
          return null;
        })()
      )}

      {/* Quick Actions */}
      <div className="mb-4 flex gap-2 flex-wrap">
        <button
          onClick={() => handleControl('start')}
          disabled={!!actionLoading}
          className="px-3 py-1 bg-green-600 hover:bg-green-700 rounded text-sm disabled:opacity-50"
        >
          Start All
        </button>
        <button
          onClick={() => handleControl('stop')}
          disabled={!!actionLoading}
          className="px-3 py-1 bg-red-600 hover:bg-red-700 rounded text-sm disabled:opacity-50"
        >
          Stop All
        </button>
        <button
          onClick={() => handleControl('restart')}
          disabled={!!actionLoading}
          className="px-3 py-1 bg-yellow-600 hover:bg-yellow-700 rounded text-sm disabled:opacity-50"
        >
          Restart All
        </button>
      </div>

      {/* Container Status */}
      <div className="mb-4">
        <h4 className="text-sm font-semibold mb-2">Containers</h4>
        <div className="space-y-2">
          {status?.containers.map((container, idx) => (
            <div key={idx} className="flex items-center justify-between p-2 bg-gray-800 rounded">
              <div className="flex items-center gap-2">
                <div className={`w-2 h-2 rounded-full ${getStatusColor(container.State)}`} />
                <span className="text-sm">{container.Name}</span>
                <span className="text-xs text-gray-400">{container.Status}</span>
              </div>
              <div className="flex gap-1">
                <button
                  onClick={() => handleControl('restart', container.Name)}
                  disabled={!!actionLoading}
                  className="px-2 py-1 bg-yellow-600 hover:bg-yellow-700 rounded text-xs disabled:opacity-50"
                >
                  Restart
                </button>
                <button
                  onClick={() => fetchLogs(container.Name)}
                  className="px-2 py-1 bg-blue-600 hover:bg-blue-700 rounded text-xs"
                >
                  Logs
                </button>
              </div>
            </div>
          ))}
          {status?.containers.length === 0 && (
            <div className="text-sm text-gray-400">No containers found</div>
          )}
        </div>
      </div>

      {/* ROS Topics */}
      <div className="mb-4">
        <h4 className="text-sm font-semibold mb-2">ROS Topics</h4>
        <div className="space-y-1">
          {status?.topics.map((topic, idx) => (
            <div key={idx} className="text-sm text-gray-300 font-mono bg-gray-800 p-2 rounded">
              {topic}
            </div>
          ))}
          {status?.topics.length === 0 && (
            <div className="text-sm text-gray-400">No topics available (containers may not be running)</div>
          )}
        </div>
      </div>

      {/* Logs */}
      {Object.keys(logs).length > 0 && (
        <div className="mt-4">
          <h4 className="text-sm font-semibold mb-2">Logs</h4>
          {Object.entries(logs).map(([service, serviceLogs]) => (
            <div key={service} className="mb-2">
              <div className="text-xs text-gray-400 mb-1">{service}</div>
              <div className="bg-black p-2 rounded font-mono text-xs text-green-400 max-h-40 overflow-y-auto">
                {serviceLogs.slice(-20).map((log, idx) => (
                  <div key={idx}>{log}</div>
                ))}
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default ROSManagementPanel;

