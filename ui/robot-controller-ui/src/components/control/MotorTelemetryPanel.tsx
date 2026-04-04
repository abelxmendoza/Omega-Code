/*
# File: /src/components/control/MotorTelemetryPanel.tsx
# Summary:
# Motor telemetry display — Front Left, Front Right, Rear Left, Rear Right.
# Reads live data from CommandContext (which polls via the shared movement WS).
# Displays speed (RPM), power (W), and PWM for each motor.
*/

import React, { useState, memo, useMemo } from 'react';
import { useCommand } from '@/context/CommandContext';
import MovementV2Modal from './MovementV2Modal';

type ServerStatus = 'connecting' | 'connected' | 'disconnected' | 'error';

const StatusDot = memo(({ status, title }: { status: ServerStatus; title: string }) => {
  const color =
    status === 'connected'    ? 'bg-emerald-500'
    : status === 'connecting' ? 'bg-slate-500'
    : status === 'error'      ? 'bg-red-600'
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
StatusDot.displayName = 'StatusDot';

interface MotorCardProps {
  label: string;
  accent: string;
  speed: number;
  power: number;
  pwm: number;
}

const MotorCard = memo(({ label, accent, speed, power, pwm }: MotorCardProps) => (
  <div className="bg-gray-700 p-1.5 rounded">
    <div className={`text-xs font-medium ${accent} mb-1`}>{label}</div>
    <div className="space-y-1 text-xs">
      <div className="flex justify-between">
        <span className="text-gray-400">Speed:</span>
        <span className="text-white font-mono">{speed.toFixed(1)} RPM</span>
      </div>
      <div className="flex justify-between">
        <span className="text-gray-400">Power:</span>
        <span className="text-white font-mono">{power.toFixed(1)}W</span>
      </div>
      <div className="flex justify-between">
        <span className="text-gray-400">PWM:</span>
        <span className="text-white font-mono">{pwm}</span>
      </div>
    </div>
  </div>
));
MotorCard.displayName = 'MotorCard';

const ZERO = { speed: 0, power: 0, pwm: 0 };

const MotorTelemetryPanel: React.FC = memo(() => {
  const { motorTelemetry, movementV2, sendCommand, status } = useCommand();

  const [isModalOpen, setIsModalOpen] = useState(false);

  const fl = motorTelemetry?.frontLeft  ?? ZERO;
  const fr = motorTelemetry?.frontRight ?? ZERO;
  const rl = motorTelemetry?.rearLeft   ?? ZERO;
  const rr = motorTelemetry?.rearRight  ?? ZERO;

  const wsStatus: ServerStatus = status === 'connected'
    ? 'connected'
    : status === 'connecting'
    ? 'connecting'
    : 'disconnected';

  return (
    <div className="bg-gray-800 text-white p-3 rounded-md shadow-md w-full max-w-xs flex flex-col">
      {/* Header */}
      <div className="w-full flex items-center justify-between mb-3">
        <h2 className="text-base font-bold">Motor Telemetry</h2>
        <div className="flex items-center gap-2">
          <button
            onClick={() => setIsModalOpen(true)}
            className="text-xs px-2 py-0.5 rounded bg-purple-600/30 text-purple-300 hover:bg-purple-600/50 transition-colors"
            title="Open Movement V2 Status"
          >
            Status
          </button>
          <StatusDot status={wsStatus} title={`Motor telemetry: ${wsStatus}`} />
        </div>
      </div>

      {/* Motor cards */}
      <div className="w-full mb-3">
        <div className="grid grid-cols-2 gap-1.5">
          <MotorCard label="Front Left"  accent="text-blue-300"   {...fl} />
          <MotorCard label="Front Right" accent="text-green-300"  {...fr} />
          <MotorCard label="Rear Left"   accent="text-purple-300" {...rl} />
          <MotorCard label="Rear Right"  accent="text-orange-300" {...rr} />
        </div>

        {/* Profile switcher */}
        <div className="mt-3 pt-3 border-t border-gray-700">
          <div className="flex items-center justify-between mb-2">
            <span className="text-xs text-gray-400">Profile:</span>
            <span className={`text-xs font-medium capitalize ${
              status === 'connected' ? 'text-purple-300' : 'text-gray-500'
            }`}>
              {status === 'connected' ? (movementV2?.profile?.name || 'Unknown') : 'Server Offline'}
            </span>
          </div>
          <div className="flex gap-1.5">
            {(['smooth', 'aggressive', 'precision'] as const).map((profile) => (
              <button
                key={profile}
                onClick={() => {
                  if (status === 'connected') {
                    sendCommand('set-profile', { profile });
                    setTimeout(() => sendCommand('status'), 300);
                  }
                }}
                disabled={status !== 'connected'}
                className={`flex-1 px-2 py-1 text-xs rounded transition-all ${
                  status === 'connected'
                    ? movementV2?.profile?.name === profile
                      ? 'bg-purple-600/30 text-purple-300 border border-purple-600/50'
                      : 'bg-gray-700 text-gray-400 hover:bg-gray-600 hover:text-gray-300 border border-gray-600'
                    : 'bg-gray-800 text-gray-600 border border-gray-700 cursor-not-allowed opacity-50'
                }`}
                title={status === 'connected' ? `Switch to ${profile} profile` : 'Server not connected'}
              >
                {profile.charAt(0).toUpperCase() + profile.slice(1)}
              </button>
            ))}
          </div>
        </div>

        {/* Connection status */}
        <div className="mt-2 flex items-center justify-between text-xs">
          <span className="text-gray-400">Status:</span>
          <span className="text-gray-300">{status}</span>
        </div>
      </div>

      {/* Movement V2 modal */}
      <MovementV2Modal
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
        movementV2={movementV2 || null}
      />
    </div>
  );
});
MotorTelemetryPanel.displayName = 'MotorTelemetryPanel';

export default MotorTelemetryPanel;
