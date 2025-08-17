/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/ServiceStatusBar.tsx
# Summary:
#   Compact, live status bar for backend services:
#     - Movement   (WebSocket; JSON heartbeat)
#     - Ultrasonic (WebSocket; streaming JSON, no pong)
#     - Line       (WebSocket; JSON heartbeat)
#     - Lighting   (WebSocket; JSON heartbeat)
#     - Video      (HTTP; probes lightweight `/health` instead of the MJPEG stream)
#
#   It respects the active network profile (LAN | TAILSCALE | LOCAL) and picks
#   the corresponding env var URL per service. Each pill shows:
#     • connection state (connected / connecting / disconnected)
#     • optional latency (ms) when heartbeat/ping is available
#
#   Notes:
#   - WS heartbeats: services that reply to {type:"ping"}→{type:"pong"} report latency.
#   - Ultrasonic streams JSON periodically; we mark it alive on any message.
#   - Video is probed via GET/HEAD to `/health` (cheap 200/503) to avoid touching the MJPEG.
*/


'use client';

import React, { useMemo } from 'react';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';

// -----------------------------
// Profile selection + env pick
// -----------------------------
type NetProfile = 'LAN' | 'TAILSCALE' | 'LOCAL';
const PROFILE = ((process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toUpperCase() as NetProfile);

/** Pick the correct value for the active profile, with sensible fallbacks. */
const pick = (lan?: string, tailscale?: string, local?: string) =>
  PROFILE === 'LAN'       ? (lan ?? local ?? '') :
  PROFILE === 'TAILSCALE' ? (tailscale ?? local ?? '') :
                             (local ?? lan ?? tailscale ?? '');

/** Convert an MJPEG URL (.../video_feed[?]) to its sibling `/health` endpoint. */
const toHealthUrl = (videoUrl?: string) => {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
};

// -----------------------------
// Endpoints from env (per profile)
// -----------------------------
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

// -----------------------------
// Visual helpers
// -----------------------------
/** Dot supports the extra "no_camera" state for video. */
const Dot: React.FC<{state: ServiceStatus | HttpStatus}> = ({ state }) => {
  const color =
    state === 'connected'   ? 'bg-emerald-500' :
    state === 'connecting'  ? 'bg-amber-400'  :
    state === 'no_camera'   ? 'bg-sky-500'    :
                              'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} aria-hidden />;
};

const Item: React.FC<{
  label: string;
  state: ServiceStatus | HttpStatus;
  latency?: number | null;
}> = ({label, state, latency}) => (
  <div
    className="flex items-center gap-2 text-xs text-white/90"
    title={`${label}: ${state}${latency!=null?` • ${latency} ms`:''}`}
  >
    <Dot state={state} />
    <span className="font-medium">{label}</span>
    <span className="text-white/60">{state}</span>
    {latency != null && <span className="text-white/50">• {latency} ms</span>}
  </div>
);

// -----------------------------
// Hook wrappers (graceful when URL is empty)
// -----------------------------
const useMaybeWs = (url: string, opts: Parameters<typeof useWsStatus>[1]) =>
  url ? useWsStatus(url, opts) : { status: 'disconnected' as ServiceStatus, latency: null };

const useMaybeHttp = (url: string, opts: Parameters<typeof useHttpStatus>[1]) =>
  url ? useHttpStatus(url, opts) : { status: 'disconnected' as HttpStatus, latency: null };

// -----------------------------
// Component
// -----------------------------
const ServiceStatusBar: React.FC = () => {
  const VIDEO_HEALTH = useMemo(() => toHealthUrl(VIDEO), [VIDEO]);

  // WS with pong → latency
  const move  = useMaybeWs(MOVE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useMaybeWs(LINE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useMaybeWs(LIGHT, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // WS streaming (any message alive)
  const ultra = useMaybeWs(ULTRA, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  // HTTP /health with special mapping (503+placeholder → "no_camera")
  const video = useMaybeHttp(VIDEO_HEALTH, {
    intervalMs: 5000,
    timeoutMs: 2500,
    treat503AsConnecting: true,
  });

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
