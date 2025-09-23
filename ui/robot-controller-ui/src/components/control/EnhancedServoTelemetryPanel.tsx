/*
# File: /src/components/control/EnhancedServoTelemetryPanel.tsx
# Summary:
# Enhanced servo telemetry display component showing detailed servo information
# - Horizontal and Vertical servo angles, PWM values, frequency
# - Real-time WebSocket connection for live data
# - Color-coded servo cards for easy identification
# - Connection status and refresh functionality
*/

import React, { useState, useEffect } from 'react';
import { useCommand } from '@/context/CommandContext';
import { statusColor } from '@/constants/status';

type ServerStatus = 'connecting' | 'connected' | 'disconnected' | 'error';
type ServoTelemetrySource = 'ack' | 'status' | null;

interface ServoData {
  horizontal: {
    angle: number;
    pwm: number;
    frequency: number;
    min: number;
    max: number;
  };
  vertical: {
    angle: number;
    pwm: number;
    frequency: number;
    min: number;
    max: number;
  };
  updatedAt: number | null;
  source: ServoTelemetrySource;
}

// Small connection status dot
function StatusDot({ status, title }: { status: ServerStatus; title: string }) {
  const color =
    status === 'connected' ? 'bg-emerald-500'
    : status === 'connecting' ? 'bg-slate-500'
    : status === 'error' ? 'bg-red-600'
    : 'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
}

const formatAngle = (value: number | null): string =>
  value == null ? '—' : `${Math.round(value)}°`;

const formatPWM = (angle: number): number => {
  // Convert angle to PWM value based on servo control logic
  // Horizontal: 2500 - int((angle + error) / 0.09)
  // Vertical: 500 + int((angle + error) / 0.09)
  return Math.round(angle * 11.11); // Approximate conversion
};

const formatRange = (min: number | null, max: number | null): string => {
  if (min == null || max == null) return '—';
  return `${Math.round(min)}° – ${Math.round(max)}°`;
};

const sourceLabel: Record<Exclude<ServoTelemetrySource, null>, string> = {
  ack: 'Ack',
  status: 'Status',
};

const statusLabel = (status: ServerStatus) =>
  status === 'connected' ? 'Live' : status === 'connecting' ? 'Connecting' : 'Offline';

const EnhancedServoTelemetryPanel: React.FC = () => {
  const { servoTelemetry, status, requestStatus } = useCommand();
  const { horizontal, vertical, min, max, updatedAt, source } = servoTelemetry;

  // Enhanced servo data with calculated values
  const [servoData, setServoData] = useState<ServoData>({
    horizontal: {
      angle: horizontal || 90,
      pwm: formatPWM(horizontal || 90),
      frequency: 50, // Standard servo frequency
      min: min || 0,
      max: max || 180
    },
    vertical: {
      angle: vertical || 90,
      pwm: formatPWM(vertical || 90),
      frequency: 50,
      min: min || 0,
      max: max || 180
    },
    updatedAt: updatedAt,
    source: source
  });

  // Update servo data when telemetry changes
  useEffect(() => {
    setServoData({
      horizontal: {
        angle: horizontal || 90,
        pwm: formatPWM(horizontal || 90),
        frequency: 50,
        min: min || 0,
        max: max || 180
      },
      vertical: {
        angle: vertical || 90,
        pwm: formatPWM(vertical || 90),
        frequency: 50,
        min: min || 0,
        max: max || 180
      },
      updatedAt: updatedAt,
      source: source
    });
  }, [horizontal, vertical, min, max, updatedAt, source]);

  const updatedText = React.useMemo(() => {
    if (!updatedAt) return 'Waiting for telemetry…';
    const time = new Date(updatedAt).toLocaleTimeString();
    const via = source ? ` via ${sourceLabel[source]}` : '';
    return `Updated ${time}${via}`;
  }, [updatedAt, source]);

  const canRefresh = status === 'connected';

  const handleRefresh = React.useCallback(() => {
    if (!canRefresh) return;
    requestStatus('servo refresh');
  }, [canRefresh, requestStatus]);

  return (
    <div className="bg-gray-800 text-white p-3 rounded-md shadow-md w-full max-w-xs flex flex-col">
      {/* Header with status dot */}
      <div className="w-full flex items-center justify-between mb-3">
        <h2 className="text-base font-bold">Servo Telemetry</h2>
        <StatusDot 
          status={status} 
          title={`Servo telemetry: ${status}`} 
        />
      </div>

      {/* Servo Status Grid */}
      <div className="w-full mb-3">
        <div className="grid grid-cols-2 gap-1.5">
          {/* Horizontal Servo */}
          <div className="bg-gray-700 p-1.5 rounded">
            <div className="text-xs font-medium text-blue-300 mb-1">Horizontal (Pan)</div>
            <div className="space-y-1 text-xs">
              <div className="flex justify-between">
                <span className="text-gray-400">Angle:</span>
                <span className="text-white font-mono">{formatAngle(servoData.horizontal.angle)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">PWM:</span>
                <span className="text-white font-mono">{servoData.horizontal.pwm}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Freq:</span>
                <span className="text-white font-mono">{servoData.horizontal.frequency}Hz</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Range:</span>
                <span className="text-white font-mono">{formatRange(servoData.horizontal.min, servoData.horizontal.max)}</span>
              </div>
            </div>
          </div>

          {/* Vertical Servo */}
          <div className="bg-gray-700 p-1.5 rounded">
            <div className="text-xs font-medium text-green-300 mb-1">Vertical (Tilt)</div>
            <div className="space-y-1 text-xs">
              <div className="flex justify-between">
                <span className="text-gray-400">Angle:</span>
                <span className="text-white font-mono">{formatAngle(servoData.vertical.angle)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">PWM:</span>
                <span className="text-white font-mono">{servoData.vertical.pwm}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Freq:</span>
                <span className="text-white font-mono">{servoData.vertical.frequency}Hz</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-400">Range:</span>
                <span className="text-white font-mono">{formatRange(servoData.vertical.min, servoData.vertical.max)}</span>
              </div>
            </div>
          </div>
        </div>
        
        {/* Connection Status and Refresh */}
        <div className="mt-3 flex items-center justify-between text-xs">
          <span className="text-gray-400">{updatedText}</span>
          <button
            type="button"
            onClick={handleRefresh}
            disabled={!canRefresh}
            className={`rounded-md px-2 py-1 font-medium transition-colors disabled:cursor-not-allowed disabled:opacity-50 ${
              canRefresh ? 'bg-emerald-600 text-white hover:bg-emerald-500' : 'bg-zinc-700 text-zinc-300'
            }`}
          >
            Refresh
          </button>
        </div>
      </div>
    </div>
  );
};

export default EnhancedServoTelemetryPanel;
