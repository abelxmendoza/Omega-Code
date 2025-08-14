/*
# File: /src/components/Header.tsx
# Summary:
Header with live per-service status (movement, ultrasonic, line tracker, lighting)
and battery bar. Uses env profile (lan | tailscale | local) to pick WS URLs.
*/

import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';

interface HeaderProps {
  // NOTE: Avoid passing isConnected from the parent; we compute status from services.
  batteryLevel: number;
}

/** Resolve endpoint from profile */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local';
  return (
    (process.env as any)[`${base}_${profile.toUpperCase()}`] ||
    (process.env as any)[`${base}_LOCAL`] ||
    ''
  );
};

const MOVE  = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');
const ULTRA = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');
const LINE  = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER');
const LIGHT = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING');

const Dot: React.FC<{state: ServiceStatus}> = ({ state }) => {
  const color =
    state === 'connected' ? 'bg-emerald-500' :
    state === 'connecting' ? 'bg-amber-400' : 'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} />;
};

/** Compact pill: glyph instead of long text */
const Pill: React.FC<{
  label: string;
  state: ServiceStatus;
  latency?: number | null;
}> = ({ label, state, latency }) => {
  const glyph = state === 'connected' ? '✓' : state === 'connecting' ? '…' : '×';
  const glyphClass =
    state === 'connected' ? 'text-emerald-400' :
    state === 'connecting' ? 'text-amber-300' : 'text-rose-400';

  return (
    <div
      className="flex items-center gap-1.5 text-[11px] px-2 py-1 rounded-md bg-black/30 border border-white/10"
      title={`${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`}
    >
      <Dot state={state} />
      <span className="text-white/90">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>
        {glyph}
      </span>
      {latency != null && (
        <span className="text-white/50 ml-0.5">{latency} ms</span>
      )}
    </div>
  );
};

const Header: React.FC<HeaderProps> = ({ batteryLevel }) => {
  // Lighting + Line tracker support JSON pong → measure latency
  const light = useWsStatus(LIGHT, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useWsStatus(LINE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Ultrasonic & Movement may just stream → treat any message as alive
  const ultra = useWsStatus(ULTRA, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });
  const move  = useWsStatus(MOVE,  { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  const states: ServiceStatus[] = [move.status, ultra.status, line.status, light.status];
  const upCount = states.filter(s => s === 'connected').length;
  const overallConnected = upCount === states.length ? true : upCount > 0;

  const batteryClass = batteryLevel > 75 ? 'bg-green-500'
                    : batteryLevel > 50 ? 'bg-yellow-500'
                    : batteryLevel > 20 ? 'bg-blue-500 neon-blue'
                    : 'bg-red-500';

  return (
    <div className="flex flex-col gap-2 bg-gray-800 text-white p-4 sticky top-0 z-10 shadow-md">
      <div className="flex justify-between items-center">
        <div className="text-lg font-bold">Robot Controller</div>

        {/* Overall status + battery */}
        <div className="flex items-center space-x-4">
          <div className="flex items-center text-sm">
            <span className="opacity-80">Status:</span>
            {overallConnected ? (
              <FaCheckCircle data-testid="status-icon" className="text-green-500 ml-2" />
            ) : (
              <FaTimesCircle data-testid="status-icon" className="text-red-500 ml-2" />
            )}
            <span className="ml-2 opacity-80">
              {upCount}/{states.length} online
            </span>
          </div>

          <div className="flex items-center text-sm">
            <span className="opacity-80">Battery:</span>
            <div className="ml-2 w-32 battery-container">
              <div className={`h-4 rounded ${batteryClass}`} style={{ width: `${batteryLevel}%` }} />
            </div>
            <span className="ml-2 opacity-80">{batteryLevel}%</span>
          </div>
        </div>
      </div>

      {/* Per-service pills (compact) */}
      <div className="flex flex-wrap gap-2">
        <Pill label="Movement"   state={move.status} />
        <Pill label="Ultrasonic" state={ultra.status} />
        <Pill label="Line"       state={line.status}   latency={line.latency} />
        <Pill label="Lighting"   state={light.status}  latency={light.latency} />
      </div>
    </div>
  );
};

export default Header;
