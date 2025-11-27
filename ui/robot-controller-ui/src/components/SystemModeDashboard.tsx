/*
System Mode Dashboard Component

Provides UI for controlling system modes (0-7) with real-time status updates.
*/

'use client';

import React, { useState, useEffect, useCallback } from 'react';
import { Activity, Cpu, Thermometer, AlertTriangle, CheckCircle, XCircle } from 'lucide-react';

interface SystemModeStatus {
  mode: number;
  description: string;
  manual_override: boolean;
  mode_name: string;
  hybrid_mode?: string;
  orin_available?: boolean;
  thermal_temp?: number;
  cpu_load?: number;
  throttling?: boolean;
}

interface SystemMode {
  mode: number;
  name: string;
  description: string;
  available: boolean;
}

interface SystemModesResponse {
  ok: boolean;
  modes: Record<number, SystemMode>;
  current_mode: number;
}

const MODE_NAMES: Record<number, string> = {
  0: 'Camera Only',
  1: 'Motion Detection',
  2: 'Tracking',
  3: 'Face Detection',
  4: 'ArUco Detection',
  5: 'Recording Only',
  6: 'Orin-Enhanced',
  7: 'Orin Navigation',
};

const MODE_COLORS: Record<number, string> = {
  0: 'bg-gray-600',
  1: 'bg-blue-600',
  2: 'bg-green-600',
  3: 'bg-purple-600',
  4: 'bg-yellow-600',
  5: 'bg-red-600',
  6: 'bg-indigo-600',
  7: 'bg-pink-600',
};

export default function SystemModeDashboard() {
  const [status, setStatus] = useState<SystemModeStatus | null>(null);
  const [modes, setModes] = useState<Record<number, SystemMode>>({});
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const fetchStatus = useCallback(async () => {
    try {
      const response = await fetch('/api/system/mode/status');
      if (!response.ok) throw new Error('Failed to fetch status');
      const data = await response.json();
      setStatus(data);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    }
  }, []);

  const fetchModes = useCallback(async () => {
    try {
      const response = await fetch('/api/system/mode/list');
      if (!response.ok) throw new Error('Failed to fetch modes');
      const data: SystemModesResponse = await response.json();
      setModes(data.modes);
    } catch (err) {
      console.error('Failed to fetch modes:', err);
    }
  }, []);

  const setMode = useCallback(async (mode: number) => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch('/api/system/mode/set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
      });
      
      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Failed to set mode');
      }
      
      const data = await response.json();
      setStatus(data);
      
      // Refresh status after a short delay
      setTimeout(fetchStatus, 500);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to set mode');
    } finally {
      setLoading(false);
    }
  }, [fetchStatus]);

  useEffect(() => {
    fetchModes();
    fetchStatus();
    
    // Poll status every second
    const interval = setInterval(fetchStatus, 1000);
    return () => clearInterval(interval);
  }, [fetchStatus, fetchModes]);

  const currentMode = status?.mode ?? 0;
  const isThrottling = status?.throttling ?? false;
  const thermalTemp = status?.thermal_temp ?? 0;
  const cpuLoad = status?.cpu_load ?? 0;

  return (
    <div className="bg-gray-900 rounded-lg p-6 shadow-lg">
      <h2 className="text-2xl font-bold text-white mb-4 flex items-center gap-2">
        <Activity className="w-6 h-6" />
        System Mode Control
      </h2>

      {/* Current Status */}
      {status && (
        <div className="mb-6 p-4 bg-gray-800 rounded-lg">
          <div className="flex items-center justify-between mb-2">
            <div>
              <span className="text-gray-400 text-sm">Current Mode</span>
              <div className="flex items-center gap-2 mt-1">
                <span className={`px-3 py-1 rounded text-white font-semibold ${MODE_COLORS[currentMode]}`}>
                  Mode {currentMode}: {MODE_NAMES[currentMode]}
                </span>
                {status.manual_override && (
                  <span className="text-xs bg-yellow-600 text-white px-2 py-1 rounded">
                    Manual Override
                  </span>
                )}
              </div>
            </div>
            {status.hybrid_mode && (
              <div className="text-right">
                <span className="text-gray-400 text-sm">Hybrid Mode</span>
                <div className="text-white font-semibold capitalize">
                  {status.hybrid_mode.replace('_', ' ')}
                </div>
              </div>
            )}
          </div>
          
          <p className="text-gray-300 text-sm mt-2">{status.description}</p>

          {/* Thermal/CPU Status */}
          <div className="flex gap-4 mt-4 pt-4 border-t border-gray-700">
            <div className="flex items-center gap-2">
              <Thermometer className="w-4 h-4 text-orange-500" />
              <span className="text-gray-400 text-sm">Temp:</span>
              <span className={`text-sm font-semibold ${thermalTemp > 70 ? 'text-red-500' : thermalTemp > 60 ? 'text-yellow-500' : 'text-green-500'}`}>
                {thermalTemp.toFixed(1)}°C
              </span>
            </div>
            <div className="flex items-center gap-2">
              <Cpu className="w-4 h-4 text-blue-500" />
              <span className="text-gray-400 text-sm">CPU:</span>
              <span className={`text-sm font-semibold ${cpuLoad > 75 ? 'text-red-500' : cpuLoad > 50 ? 'text-yellow-500' : 'text-green-500'}`}>
                {cpuLoad.toFixed(1)}%
              </span>
            </div>
            {isThrottling && (
              <div className="flex items-center gap-2 ml-auto">
                <AlertTriangle className="w-4 h-4 text-yellow-500" />
                <span className="text-yellow-500 text-sm font-semibold">Throttling Active</span>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Error Display */}
      {error && (
        <div className="mb-4 p-3 bg-red-900/50 border border-red-500 rounded-lg flex items-center gap-2">
          <XCircle className="w-5 h-5 text-red-500" />
          <span className="text-red-200 text-sm">{error}</span>
        </div>
      )}

      {/* Mode Buttons */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
        {[0, 1, 2, 3, 4, 5, 6, 7].map((mode) => {
          const modeInfo = modes?.[mode];
          const isActive = currentMode === mode;
          const isAvailable = modeInfo?.available !== false;
          
          return (
            <button
              key={mode}
              onClick={() => isAvailable && !loading && setMode(mode)}
              disabled={loading || !isAvailable}
              className={`
                p-4 rounded-lg border-2 transition-all
                ${isActive 
                  ? `${MODE_COLORS[mode]} border-white text-white` 
                  : 'bg-gray-800 border-gray-700 text-gray-300 hover:border-gray-600'
                }
                ${!isAvailable ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}
                ${loading ? 'opacity-50 cursor-wait' : ''}
              `}
              title={modeInfo?.description || MODE_NAMES[mode]}
            >
              <div className="text-center">
                <div className="text-2xl font-bold mb-1">{mode}</div>
                <div className="text-xs font-semibold">{MODE_NAMES[mode]}</div>
                {isActive && (
                  <CheckCircle className="w-4 h-4 mx-auto mt-2" />
                )}
              </div>
            </button>
          );
        })}
      </div>

      {/* Loading Indicator */}
      {loading && (
        <div className="mt-4 text-center text-gray-400 text-sm">
          Setting mode...
        </div>
      )}
    </div>
  );
}

