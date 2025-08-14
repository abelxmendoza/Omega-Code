'use client';
/*
Header with live per-service status (movement, ultrasonic, line, lighting, video)
and battery bar. Uses env profile (lan|tailscale|local) to pick URLs.
*/

import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';

interface HeaderProps { batteryLevel: number; }

/** Explicit env picking (Next.js requires dot-access at build time) */
type NetProfile = 'LAN' | 'TAILSCALE' | 'LOCAL';
const PROFILE = ((process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toUpperCase() as NetProfile);

const pick = (lan?: string, tailscale?: string, local?: string) =>
  PROFILE === 'LAN'       ? (lan ?? local ?? '') :
  PROFILE === 'TAILSCALE' ? (tailscale ?? local ?? '') :
                             (local ?? lan ?? tailscale ?? '');

const MOVE  = pick(
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE,
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL
);
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

type Status = 'connected' | 'connecting' | 'disconnected';

const Dot: React.FC<{state: Status}> = ({ state }) => {
  const color =
    state === 'connected' ? 'bg-emerald-500' :
    state === 'connecting' ? 'bg-amber-400' : 'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} aria-hidden />;
};

const Pill: React.FC<{ label: string; state: Status; latency?: number|null }> = ({ label, state, latency }) => {
  const glyph = state === 'connected' ? '✓' : state === 'connecting' ? '…' : '×';
  const glyphClass =
    state === 'connected' ? 'text-emerald-400' :
    state === 'connecting' ? 'text-amber-300' : 'text-rose-400';

  return (
    <div className="flex items-center gap-1.5 text-[11px] px-2 py-1 rounded-md bg-black/30 border border-white/10"
         title={`${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`}>
      <Dot state={state} />
      <span className="text-white/90">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </div>
  );
};

const useMaybeWs = (url: string, opts: Parameters<typeof useWsStatus>[1]) =>
  url ? useWsStatus(url, opts) : { status: 'disconnected' as ServiceStatus, latency: null };

const useMaybeHttp = (url: string, opts: Parameters<typeof useHttpStatus>[1]) =>
  url ? useHttpStatus(url, opts) : { status: 'disconnected' as HttpStatus, latency: null };

const Header: React.FC<HeaderProps> = ({ batteryLevel }) => {
  // WS with pong → latency
  const move  = useMaybeWs(MOVE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useMaybeWs(LINE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useMaybeWs(LIGHT, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Streaming WS (no pong)
  const ultra = useMaybeWs(ULTRA, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  // HTTP MJPEG probe
  const video = useMaybeHttp(VIDEO, { intervalMs: 5000, timeoutMs: 2500 });

  const states: Status[] = [move.status, ultra.status, line.status, light.status, video.status];
  const upCount = states.filter(s => s === 'connected').length;
  const overallConnected = upCount === states.length ? true : upCount > 0;

  const batteryClass =
    batteryLevel > 75 ? 'bg-green-500' :
    batteryLevel > 50 ? 'bg-yellow-500' :
    batteryLevel > 20 ? 'bg-blue-500 neon-blue' :
    'bg-red-500';

  return (
    <div className="flex flex-col gap-2 bg-gray-800 text-white p-4 sticky top-0 z-10 shadow-md">
      <div className="flex justify-between items-center">
        <div className="text-lg font-bold">Robot Controller</div>

        {/* Overall status + battery */}
        <div className="flex items-center space-x-4">
          <div className="flex items-center text-sm">
            <span className="opacity-80">Status:</span>
            {overallConnected ? (
              <FaCheckCircle aria-label="All services reachable" className="text-green-500 ml-2" />
            ) : (
              <FaTimesCircle aria-label="Some services down" className="text-red-500 ml-2" />
            )}
            <span className="ml-2 opacity-80">{upCount}/{states.length} online</span>
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

      {/* Per-service pills */}
      <div className="flex flex-wrap gap-2">
        <Pill label="Movement"    state={move.status} />
        <Pill label="Ultrasonic"  state={ultra.status} />
        <Pill label="Line"        state={line.status}   latency={line.latency} />
        <Pill label="Lighting"    state={light.status}  latency={light.latency} />
        <Pill label="Video"       state={video.status}  latency={video.latency} />
      </div>
    </div>
  );
};

export default Header;
