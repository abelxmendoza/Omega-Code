'use client';
import React from 'react';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus } from '../hooks/useHttpStatus';

type NetProfile = 'LAN' | 'TAILSCALE' | 'LOCAL';
const PROFILE = ((process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toUpperCase() as NetProfile);
const pick = (lan?: string, tailscale?: string, local?: string) =>
  PROFILE === 'LAN'       ? (lan ?? local ?? '') :
  PROFILE === 'TAILSCALE' ? (tailscale ?? local ?? '') :
                             (local ?? lan ?? tailscale ?? '');

const ULTRA = pick(
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL
);
const LINE  = pick(
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL
);
const MOVE  = pick(
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL
);
const LIGHT = pick(
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL
);
const VIDEO = pick(
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
);

const Dot: React.FC<{state: 'connected'|'connecting'|'disconnected'}> = ({ state }) => {
  const color =
    state === 'connected' ? 'bg-emerald-500' :
    state === 'connecting' ? 'bg-amber-400' : 'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} aria-hidden />;
};

const Item: React.FC<{label: string; state: 'connected'|'connecting'|'disconnected'; latency?: number|null}> =
({label, state, latency}) => (
  <div className="flex items-center gap-2 text-xs text-white/90" title={`${label}: ${state}${latency!=null?` • ${latency} ms`:''}`}>
    <Dot state={state} />
    <span className="font-medium">{label}</span>
    <span className="text-white/60">{state}</span>
    {latency != null && <span className="text-white/50">• {latency} ms</span>}
  </div>
);

const useMaybeWs = (url: string, opts: Parameters<typeof useWsStatus>[1]) =>
  url ? useWsStatus(url, opts) : { status: 'disconnected' as ServiceStatus, latency: null };

const ServiceStatusBar: React.FC = () => {
  const move  = useMaybeWs(MOVE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useMaybeWs(LINE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useMaybeWs(LIGHT, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const ultra = useMaybeWs(ULTRA, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  const video = useHttpStatus(VIDEO, { intervalMs: 5000, timeoutMs: 2500 });

  const states = [move.status, ultra.status, line.status, light.status, video.status] as const;
  const up = states.filter(s => s === 'connected').length;

  return (
    <div className="flex flex-col gap-2">
      <div className="text-xs text-white/70" aria-live="polite">
        Services online: {up}/{states.length}
      </div>
      <div className="flex flex-wrap items-center gap-4 px-3 py-2 bg-black/40 border border-white/10 rounded-md">
        <Item label="Movement"    state={move.status}   />
        <Item label="Ultrasonic"  state={ultra.status}  />
        <Item label="Line"        state={line.status}   latency={line.latency} />
        <Item label="Lighting"    state={light.status}  latency={light.latency} />
        <Item label="Video"       state={video.status}  latency={video.latency} />
      </div>
    </div>
  );
};

export default ServiceStatusBar;
