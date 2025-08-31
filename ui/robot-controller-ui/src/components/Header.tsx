/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/Header.tsx
# Summary:
#   App header with overall health + battery and per-service status pills.
#   - Uses profile-aware resolver (`netProfile`) for all endpoints
#   - WS services (movement/line/lighting): JSON ping→pong for latency
#   - Ultrasonic: marks alive on any message (no pong)
#   - Video: probes sibling `/health` (cheap 200/503) instead of touching MJPEG
#   - Colors match ServiceStatusBar (green=connected, amber=connecting, red=disconnected, sky=no_camera)
#   - NEW: Network Wizard toggle pill + embedded <NetworkWizard /> panel
*/

'use client';

import React, { useMemo, useState } from 'react';
import dynamic from 'next/dynamic';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';
import { net } from '@/utils/netProfile';

// Lazy (client-only) load for the wizard — matches actual file location
const NetworkWizard = dynamic(() => import('@/components/NetworkWizard'), { ssr: false });

interface HeaderProps {
  batteryLevel: number;
}

/** Convert an MJPEG URL (.../video_feed[?]) to the sibling `/health`. */
function toHealthUrl(videoUrl?: string) {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
}

type HeaderState = ServiceStatus | HttpStatus; // allow 'no_camera' from http hook

const Dot: React.FC<{ state: HeaderState }> = ({ state }) => {
  const color =
    state === 'connected'   ? 'bg-emerald-500' :
    state === 'connecting'  ? 'bg-amber-400'  :
    state === 'no_camera'   ? 'bg-sky-500'    :
                              'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} aria-hidden />;
};

type PillProps = {
  label: string;
  state: HeaderState;
  latency?: number | null;
  onClick?: () => void;
  isInteractive?: boolean;
};

const Pill: React.FC<PillProps> = ({ label, state, latency, onClick, isInteractive }) => {
  const glyph =
    state === 'connected'  ? '✓' :
    state === 'connecting' ? '…' :
    state === 'no_camera'  ? '○' : '×';
  const glyphClass =
    state === 'connected'  ? 'text-emerald-400' :
    state === 'connecting' ? 'text-amber-300'  :
    state === 'no_camera'  ? 'text-sky-300'    : 'text-rose-400';

  const base =
    'flex items-center gap-1.5 text-[11px] px-2 py-1 rounded-md bg-black/30 border border-white/10';
  const interactive = isInteractive
    ? ' cursor-pointer hover:bg-black/40 focus:outline-none focus:ring-2 focus:ring-white/20'
    : '';

  const content = (
    <>
      <Dot state={state} />
      <span className="text-white/90">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </>
  );

  return isInteractive ? (
    <button type="button" className={base + interactive} onClick={onClick} title={label}>
      {content}
    </button>
  ) : (
    <div className={base} title={`${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`}>
      {content}
    </div>
  );
};

// Graceful wrappers: if a URL is empty, pretend disconnected (keeps UI stable)
const useMaybeWs = (url: string, opts: Parameters<typeof useWsStatus>[1]) =>
  url ? useWsStatus(url, opts) : { status: 'disconnected' as ServiceStatus, latency: null };

const useMaybeHttp = (url: string, opts: Parameters<typeof useHttpStatus>[1]) =>
  url ? useHttpStatus(url, opts) : { status: 'disconnected' as HttpStatus, latency: null };

const Header: React.FC<HeaderProps> = ({ batteryLevel }) => {
  const [showNetwork, setShowNetwork] = useState(false);

  // Resolve all endpoints via the profile-aware helper (honors ?profile).
  const MOVE_URL  = net.ws.movement();
  const ULTRA_URL = net.ws.ultrasonic();
  const LINE_URL  = net.ws.line();
  const LIGHT_URL = net.ws.lighting();
  const VIDEO_URL = net.video();

  // Prefer probing /health for video to avoid pulling the MJPEG stream.
  const VIDEO_HEALTH = useMemo(() => toHealthUrl(VIDEO_URL), [VIDEO_URL]);

  // WS with pong → latency
  const move  = useMaybeWs(MOVE_URL,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useMaybeWs(LINE_URL,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useMaybeWs(LIGHT_URL, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Streaming WS (any message marks alive)
  const ultra = useMaybeWs(ULTRA_URL, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  // HTTP /health mapping (503+placeholder → "no_camera")
  const video = useMaybeHttp(VIDEO_HEALTH, {
    intervalMs: 5000,
    timeoutMs: 2500,
    treat503AsConnecting: true,
  });

  const states = [move.status, ultra.status, line.status, light.status, video.status] as const;
  const upCount = states.filter((s) => s === 'connected').length;
  const allGood = upCount === states.length;

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
            {allGood ? (
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
      <div className="flex flex-wrap gap-2 items-center">
        <Pill label="Movement"   state={move.status}   latency={move.latency} />
        <Pill label="Ultrasonic" state={ultra.status} />
        <Pill label="Line"       state={line.status}   latency={line.latency} />
        <Pill label="Lighting"   state={light.status}  latency={light.latency} />
        <Pill label="Video"      state={video.status}  latency={video.latency} />

        {/* Network Wizard toggle */}
        <Pill
          label={showNetwork ? 'Network ▲' : 'Network ▼'}
          state="connected"
          isInteractive
          onClick={() => setShowNetwork((v) => !v)}
        />
      </div>

      {/* Embedded Network Wizard */}
      {showNetwork && (
        <div className="mt-2">
          <NetworkWizard />
        </div>
      )}
    </div>
  );
};

export default Header;
