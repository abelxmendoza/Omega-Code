/*
# File: /src/components/control/MotorTelemetryPanel.tsx
# Summary:
# Standalone motor telemetry display component showing all 4 motors
# - Front Left, Front Right, Rear Left, Rear Right
# - Displays speed (RPM), power (W), and PWM for each motor
# - Real-time WebSocket connection for live data
# - Color-coded motor cards for easy identification
*/

import React, { useState, memo, useMemo } from 'react';
import { useRobustWebSocket } from '@/utils/RobustWebSocket';
import { handleWebSocketError, handleComponentError } from '@/utils/errorHandling';
import { withOptimization, performanceMonitor } from '@/utils/optimization';
import { envConfig } from '@/config/environment';

type ServerStatus = 'connecting' | 'connected' | 'disconnected' | 'error';

interface MotorData {
  frontLeft: {
    speed: number;
    power: number;
    pwm: number;
  };
  frontRight: {
    speed: number;
    power: number;
    pwm: number;
  };
  rearLeft: {
    speed: number;
    power: number;
    pwm: number;
  };
  rearRight: {
    speed: number;
    power: number;
    pwm: number;
  };
}

// Optimized StatusDot component
const StatusDot = memo(({ status, title }: { status: ServerStatus; title: string }) => {
  const color =
    status === 'connected' ? 'bg-emerald-500'
    : status === 'connecting' ? 'bg-slate-500'
    : status === 'error' ? 'bg-red-600'
    : 'bg-black/20 backdrop-blur-sm border border-black/30';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
});

const MotorTelemetryPanel: React.FC = memo(() => {
  // Motor telemetry data
  const [motorData, setMotorData] = useState<MotorData>({
    frontLeft: { speed: 0, power: 0, pwm: 0 },
    frontRight: { speed: 0, power: 0, pwm: 0 },
    rearLeft: { speed: 0, power: 0, pwm: 0 },
    rearRight: { speed: 0, power: 0, pwm: 0 }
  });

  // Memoized motor cards for better performance
  const motorCards = useMemo(() => [
    // Front Left Motor
    <div key="frontLeft" className="bg-gray-700 p-1.5 rounded">
      <div className="text-xs font-medium text-blue-300 mb-1">Front Left</div>
      <div className="space-y-1 text-xs">
        <div className="flex justify-between">
          <span className="text-gray-400">Speed:</span>
          <span className="text-white font-mono">{motorData.frontLeft.speed.toFixed(1)} RPM</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">Power:</span>
          <span className="text-white font-mono">{motorData.frontLeft.power.toFixed(1)}W</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">PWM:</span>
          <span className="text-white font-mono">{motorData.frontLeft.pwm}</span>
        </div>
      </div>
    </div>,

    // Front Right Motor
    <div key="frontRight" className="bg-gray-700 p-1.5 rounded">
      <div className="text-xs font-medium text-green-300 mb-1">Front Right</div>
      <div className="space-y-1 text-xs">
        <div className="flex justify-between">
          <span className="text-gray-400">Speed:</span>
          <span className="text-white font-mono">{motorData.frontRight.speed.toFixed(1)} RPM</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">Power:</span>
          <span className="text-white font-mono">{motorData.frontRight.power.toFixed(1)}W</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">PWM:</span>
          <span className="text-white font-mono">{motorData.frontRight.pwm}</span>
        </div>
      </div>
    </div>,

    // Rear Left Motor
    <div key="rearLeft" className="bg-gray-700 p-1.5 rounded">
      <div className="text-xs font-medium text-purple-300 mb-1">Rear Left</div>
      <div className="space-y-1 text-xs">
        <div className="flex justify-between">
          <span className="text-gray-400">Speed:</span>
          <span className="text-white font-mono">{motorData.rearLeft.speed.toFixed(1)} RPM</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">Power:</span>
          <span className="text-white font-mono">{motorData.rearLeft.power.toFixed(1)}W</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">PWM:</span>
          <span className="text-white font-mono">{motorData.rearLeft.pwm}</span>
        </div>
      </div>
    </div>,

    // Rear Right Motor
    <div key="rearRight" className="bg-gray-700 p-1.5 rounded">
      <div className="text-xs font-medium text-orange-300 mb-1">Rear Right</div>
      <div className="space-y-1 text-xs">
        <div className="flex justify-between">
          <span className="text-gray-400">Speed:</span>
          <span className="text-white font-mono">{motorData.rearRight.speed.toFixed(1)} RPM</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">Power:</span>
          <span className="text-white font-mono">{motorData.rearRight.power.toFixed(1)}W</span>
        </div>
        <div className="flex justify-between">
          <span className="text-gray-400">PWM:</span>
          <span className="text-white font-mono">{motorData.rearRight.pwm}</span>
        </div>
      </div>
    </div>
  ], [motorData]);

  // WebSocket connection for motor telemetry
  const motorTelemetryWs = useRobustWebSocket({
    url: envConfig.wsUrls.movement[0] || 'ws://localhost:8081/',
    onMessage: (data) => {
      try {
        // Parse motor telemetry data for 4 motors
        if (data?.motors) {
          setMotorData({
            frontLeft: {
              speed: data.motors.frontLeft?.speed || data.motors.fl?.speed || 0,
              power: data.motors.frontLeft?.power || data.motors.fl?.power || 0,
              pwm: data.motors.frontLeft?.pwm || data.motors.fl?.pwm || 0
            },
            frontRight: {
              speed: data.motors.frontRight?.speed || data.motors.fr?.speed || 0,
              power: data.motors.frontRight?.power || data.motors.fr?.power || 0,
              pwm: data.motors.frontRight?.pwm || data.motors.fr?.pwm || 0
            },
            rearLeft: {
              speed: data.motors.rearLeft?.speed || data.motors.rl?.speed || 0,
              power: data.motors.rearLeft?.power || data.motors.rl?.power || 0,
              pwm: data.motors.rearLeft?.pwm || data.motors.rl?.pwm || 0
            },
            rearRight: {
              speed: data.motors.rearRight?.speed || data.motors.rr?.speed || 0,
              power: data.motors.rearRight?.power || data.motors.rr?.power || 0,
              pwm: data.motors.rearRight?.pwm || data.motors.rr?.pwm || 0
            }
          });
        } else if (data?.frontLeftMotor || data?.frontRightMotor || data?.rearLeftMotor || data?.rearRightMotor) {
          // Alternative data format
          setMotorData({
            frontLeft: {
              speed: data.frontLeftMotor?.speed || 0,
              power: data.frontLeftMotor?.power || 0,
              pwm: data.frontLeftMotor?.pwm || 0
            },
            frontRight: {
              speed: data.frontRightMotor?.speed || 0,
              power: data.frontRightMotor?.power || 0,
              pwm: data.frontRightMotor?.pwm || 0
            },
            rearLeft: {
              speed: data.rearLeftMotor?.speed || 0,
              power: data.rearLeftMotor?.power || 0,
              pwm: data.rearLeftMotor?.pwm || 0
            },
            rearRight: {
              speed: data.rearRightMotor?.speed || 0,
              power: data.rearRightMotor?.power || 0,
              pwm: data.rearRightMotor?.pwm || 0
            }
          });
        }
      } catch (error) {
        handleComponentError(error as Error, 'MotorTelemetryPanel', 'process-message');
      }
    },
    onError: (error) => {
      handleWebSocketError(error, { component: 'MotorTelemetryPanel' });
    }
  });

  return (
    <div className="bg-gray-800 text-white p-3 rounded-md shadow-md w-full max-w-xs flex flex-col">
      {/* Header with status dot */}
      <div className="w-full flex items-center justify-between mb-3">
        <h2 className="text-base font-bold">Motor Telemetry</h2>
        <StatusDot 
          status={motorTelemetryWs.connectionStatus} 
          title={`Motor telemetry: ${motorTelemetryWs.connectionStatus}`} 
        />
      </div>

      {/* Motor Status Grid */}
      <div className="w-full mb-3">
        <div className="grid grid-cols-2 gap-1.5">
          {motorCards}
        </div>
        
        {/* Connection Status */}
        <div className="mt-2 flex items-center justify-between text-xs">
          <span className="text-gray-400">Status:</span>
          <span className="text-gray-300">{motorTelemetryWs.connectionStatus}</span>
        </div>
      </div>
    </div>
  );
});

export default MotorTelemetryPanel;
