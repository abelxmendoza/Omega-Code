import React from 'react';
import { useWsStatus } from '../hooks/useWsStatus';

/** Same resolver you use elsewhere */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local';
  return (
    process.env[`${base}_${profile.toUpperCase()}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
};

const ULTRA = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');   // ws://.../ultrasonic
const LINE  = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER');  // ws://.../line-tracker
const MOVE  = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');      // ws://...:8081
const LIGHT = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING');      // ws://.../lighting

const Dot: React.FC<{state: 'connected'|'connecting'|'disconnected'}> = ({ state }) => {
  const color =
    state === 'connected' ? 'bg-emerald-500' :
    state === 'connecting' ? 'bg-amber-400' : 'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} />;
};

const Item: React.FC<{label: string; state: 'connected'|'connecting'|'disconnected'; latency?: number|null}> = ({label, state, latency}) => (
  <div className="flex items-center gap-2 text-xs text-white/90">
    <Dot state={state} />
    <span className="font-medium">{label}</span>
    <span className="text-white/60">{state}</span>
    {latency != null && <span className="text-white/50">• {latency} ms</span>}
  </div>
);

const ServiceStatusBar: React.FC = () => {
  // Lighting + Line tracker implement {"type":"pong"} → use heartbeat
  const light = useWsStatus(LIGHT, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useWsStatus(LINE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Ultrasonic streams JSON samples every ~1s, no pong → “any message keeps alive”
  const ultra = useWsStatus(ULTRA, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  // Movement: if your movement server supports pong, flip to heartbeat.
  const move  = useWsStatus(MOVE,  { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  return (
    <div className="flex flex-wrap items-center gap-4 px-3 py-2 bg-black/40 border border-white/10 rounded-md">
      <Item label="Movement"    state={move.status}   />
      <Item label="Ultrasonic"  state={ultra.status}  />
      <Item label="Line"        state={line.status}   latency={line.latency} />
      <Item label="Lighting"    state={light.status}  latency={light.latency} />
    </div>
  );
};

export default ServiceStatusBar;
