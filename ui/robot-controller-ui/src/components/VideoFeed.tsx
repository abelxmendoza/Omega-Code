/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/VideoFeed.tsx
# Summary:
#   Live MJPEG preview with mini GPS map + robust status:
#     - Uses /health (cheap JSON) to determine:
#         • connected (200)
#         • no_camera (503 + { placeholder:true })
#         • connecting (generic 503)
#         • disconnected (network/CORS/timeout)
#     - Status dot colors: green (connected) / amber (connecting or no_camera) / red (disconnected)
#     - Renders <img> for the MJPEG stream when connected. In `no_camera`, we *try* to render
#       the stream as well (in case the backend is serving a placeholder MJPEG); if it errors
#       we fall back to a clear overlay that says “No camera”.
#     - Mini GPS map (click to expand) retained.
#
#   Env-driven endpoints:
#     - Network profile: NEXT_PUBLIC_NETWORK_PROFILE = lan | tailscale | local
#     - Video stream base keys:
#         NEXT_PUBLIC_VIDEO_STREAM_URL_LAN
#         NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE
#         NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
#
#   Parent callback:
#     - onVideoStatusChange(ok: boolean) fires when effective “online” state changes
*/

'use client';

import React, { useEffect, useMemo, useRef, useState } from 'react';
import GpsLocation from './GpsLocation';
import { useHttpStatus } from '../hooks/useHttpStatus';

type NetProfile = 'LAN' | 'TAILSCALE' | 'LOCAL';
const PROFILE = ((process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toUpperCase() as NetProfile);

/** Pick per active profile with sensible fallbacks. */
const pick = (lan?: string, tailscale?: string, local?: string) =>
  PROFILE === 'LAN'       ? (lan ?? local ?? '') :
  PROFILE === 'TAILSCALE' ? (tailscale ?? local ?? '') :
                             (local ?? lan ?? tailscale ?? '');

/** Convert .../video_feed[?...] → sibling /health */
const toHealthUrl = (videoUrl?: string) => {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
};

/** Props */
export type VideoFeedProps = {
  onVideoStatusChange?: (ok: boolean) => void; // notify parent when video becomes (un)available
  className?: string;
  objectFit?: 'cover' | 'contain';
};

// --- Resolve endpoints from env ---
const VIDEO_STREAM = pick(
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
);
const VIDEO_HEALTH = toHealthUrl(VIDEO_STREAM);

// Status palette used elsewhere in your UI:
//  - connected    → emerald
//  - connecting   → amber
//  - disconnected → rose
//  - no_camera    → amber (but with explicit label)
type ServerStatus = 'connected' | 'connecting' | 'disconnected' | 'no_camera';

const StatusDot: React.FC<{ status: ServerStatus; title?: string }> = ({ status, title }) => {
  const color =
    status === 'connected'   ? 'bg-emerald-500' :
    status === 'disconnected'? 'bg-rose-500'    :
                               'bg-amber-400'; // connecting & no_camera
  return (
    <span
      className={`inline-block rounded-full ${color}`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
};

const VideoFeed: React.FC<VideoFeedProps> = ({
  onVideoStatusChange,
  className = '',
  objectFit = 'cover',
}) => {
  const [mapExpanded, setMapExpanded] = useState(false);

  // Probe /health (cheap + precise). Our hook returns:
  //   status ∈ { connected | connecting | disconnected | no_camera }
  //   latency: ms or null
  const { status, latency } = useHttpStatus(VIDEO_HEALTH || undefined, {
    intervalMs: 5000,
    timeoutMs: 2500,
    // keep default treat503AsConnecting: true
  });

  // Whether we should attempt to render the <img> MJPEG:
  // - Always when "connected"
  // - Also when "no_camera" (if backend streams a placeholder, it will load)
  const allowImage = status === 'connected' || status === 'no_camera';

  // Ensure the <img> cache-busts on each (re)attempt
  const srcWithBuster = useMemo(() => {
    if (!VIDEO_STREAM) return '';
    const sep = VIDEO_STREAM.includes('?') ? '&' : '?';
    return `${VIDEO_STREAM}${sep}b=${Date.now()}`;
  }, [status]); // bust whenever status changes

  // Track <img> load error so we can suppress it when placeholder isn’t available
  const [imgError, setImgError] = useState(false);
  useEffect(() => { setImgError(false); }, [srcWithBuster]);

  // Inform parent about effective “online” (green) state
  useEffect(() => {
    onVideoStatusChange?.(status === 'connected');
  }, [status, onVideoStatusChange]);

  const titleSuffix =
    latency != null ? ` • ${latency}ms` : '';

  const statusLabel =
    status === 'no_camera' ? 'No camera' :
    status.charAt(0).toUpperCase() + status.slice(1);

  const isOnline = status === 'connected';
  const showImg = allowImage && !!VIDEO_STREAM && !imgError;

  return (
    <div
      className={`relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden ${className}`}
      style={{ aspectRatio: '16 / 9' }}
    >
      {/* Tiny status pill (top-left) */}
      <div className="absolute top-2 left-2 z-20 flex items-center gap-2 px-2 py-1 bg-black/45 backdrop-blur rounded border border-white/15 text-white text-xs">
        <StatusDot status={status} title={`Video: ${statusLabel}${titleSuffix}`} />
        <span className="font-semibold">Video</span>
        <span className="text-white/80">
          {status === 'no_camera' ? 'No camera' : (latency != null ? `${latency}ms` : '… ms')}
        </span>
      </div>

      {/* Main media area */}
      <div className="absolute inset-0 bg-black">
        {showImg ? (
          <img
            src={srcWithBuster}
            alt="Live Video Feed"
            className={`w-full h-full ${objectFit === 'cover' ? 'object-cover' : 'object-contain'}`}
            onLoad={() => {/* image loaded, status from /health already accurate */}}
            onError={() => setImgError(true)}
          />
        ) : (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-2">
              {status === 'no_camera'
                ? 'No camera detected'
                : status === 'connecting'
                ? 'Connecting...'
                : 'Video feed not connected'}
            </div>
            {/* Offer retry when disconnected/errored */}
            <button
              type="button"
              className="px-2 py-1 rounded bg-black/40 hover:bg-black/55 border border-white/20 text-xs"
              onClick={() => window.location.reload()}
            >
              Retry
            </button>
          </div>
        )}
      </div>

      {/* Mini GPS map (top-right) */}
      {!mapExpanded && (
        <button
          type="button"
          className="absolute top-2 right-2 z-10 w-24 h-24 sm:w-28 sm:h-28 rounded-md overflow-hidden border border-white/15 shadow-md bg-black/30 hover:bg-black/40 transition"
          onClick={() => setMapExpanded(true)}
          title="Open GPS map"
        >
          <div className="pointer-events-none w-full h-full">
            <GpsLocation interactive={false} dummy showTrail={false} />
          </div>
        </button>
      )}

      {/* Expanded map overlay */}
      {mapExpanded && (
        <div className="absolute inset-0 z-20 bg-black/70 backdrop-blur-sm">
          <div className="absolute inset-3 rounded-lg overflow-hidden border border-white/15 shadow-lg bg-gray-900">
            <GpsLocation interactive dummy showAccuracy />
            <div className="absolute top-2 left-2 bg-amber-500/80 text-black text-xs font-semibold px-2 py-1 rounded">
              GPS (demo)
            </div>
            <button
              type="button"
              className="absolute top-2 right-2 bg-black/60 hover:bg-black/70 text-white text-xs px-2 py-1 rounded border border-white/20"
              onClick={() => setMapExpanded(false)}
            >
              Back to Video
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default VideoFeed;
