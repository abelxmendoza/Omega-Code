/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/VideoFeed.tsx
# Summary:
#   Live MJPEG preview with a mini GPS map and robust status handling.
#
#   Key features:
#   - Streams via same-origin proxy first: GET /api/video-proxy
#       • avoids mixed-content blocks when the UI is served over HTTPS
#       • honors ?profile=lan|tailscale|local and ?video=<override-url>
#   - Health probing via same-origin: GET /api/video-health
#       • reliable over HTTPS (no mixed-content/CORS issues)
#       • status semantics:
#           connected   → 200
#           no_camera   → 503 + { placeholder:true }
#           connecting  → generic 503
#           disconnected→ network/timeout/other
#   - Fallback: if proxy image fails, try the direct upstream URL when it’s safe (no mixed content)
#   - Auto cache-busting on retries/status changes; accessible, debuggable (“proxy|direct” tag)
#
#   Env inputs:
#     NEXT_PUBLIC_NETWORK_PROFILE = lan | tailscale | local
#     NEXT_PUBLIC_VIDEO_STREAM_URL_LAN
#     NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE
#     NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
#
#   Parent callback:
#     onVideoStatusChange(ok: boolean) – fires when effective “online” (green) changes
#
#   Notes:
#     • This component expects the API routes we added:
#         - /api/video-proxy     (streams MJPEG)
#         - /api/video-health    (for status/latency)
*/

'use client';

import React, { useEffect, useMemo, useRef, useState } from 'react';
import GpsLocation from './GpsLocation';
import { useHttpStatus } from '../hooks/useHttpStatus';
import { getActiveProfile } from '@/utils/resolveWsUrl';

type ServerStatus = 'connected' | 'connecting' | 'disconnected' | 'no_camera';

/* ----------------------------- helpers ------------------------------ */

/** Pick per active profile with sensible fallbacks (env-driven). */
function pickByProfile(lan?: string, tailscale?: string, local?: string): string {
  const prof = getActiveProfile(); // 'lan' | 'tailscale' | 'local'
  return prof === 'lan'
    ? (lan ?? local ?? '')
    : prof === 'tailscale'
    ? (tailscale ?? local ?? '')
    : (local ?? lan ?? tailscale ?? '');
}

/** Convert .../video_feed[?...] → sibling /health (for direct upstream). */
function toHealthUrl(videoUrl?: string) {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
}

/** Would fetching this URL be mixed-content when the UI is https? */
function willBeMixedContent(url: string): boolean {
  if (!url || typeof window === 'undefined') return false;
  if (window.location.protocol !== 'https:') return false;
  try {
    const u = new URL(url);
    return u.protocol === 'http:' && u.hostname !== window.location.hostname;
  } catch {
    return false;
  }
}

/** Get current page query param (for forwarding ?profile, ?video to the proxy). */
function getQueryParam(name: string): string | undefined {
  if (typeof window === 'undefined') return undefined;
  const v = new URLSearchParams(window.location.search).get(name);
  return v?.trim() || undefined;
}

/* ----------------------- env/direct upstream URLs ------------------- */

const DIRECT_VIDEO_STREAM = pickByProfile(
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
);
const DIRECT_VIDEO_HEALTH = toHealthUrl(DIRECT_VIDEO_STREAM);

/* ------------------------------- UI -------------------------------- */

const StatusDot: React.FC<{ status: ServerStatus; title?: string }> = ({ status, title }) => {
  const color =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'disconnected'
      ? 'bg-rose-500'
      : 'bg-amber-400'; // connecting & no_camera
  return (
    <span
      className={`inline-block rounded-full ${color}`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
};

export type VideoFeedProps = {
  onVideoStatusChange?: (ok: boolean) => void;
  className?: string;
  objectFit?: 'cover' | 'contain';
};

const VideoFeed: React.FC<VideoFeedProps> = ({
  onVideoStatusChange,
  className = '',
  objectFit = 'cover',
}) => {
  const [mapExpanded, setMapExpanded] = useState(false);
  const [imgError, setImgError] = useState(false);
  const [useDirect, setUseDirect] = useState(false); // start with proxy; switch to direct if proxy fails and direct is safe
  const [buster, setBuster] = useState<number>(() => Date.now());
  const [isOffline, setIsOffline] = useState<boolean>(typeof navigator !== 'undefined' ? !navigator.onLine : false);

  // Forward page overrides to the proxy (if present)
  const qpProfile = getQueryParam('profile'); // optional page override
  const qpVideo = getQueryParam('video'); // optional page override
  const activeProfile = (qpProfile as any) || getActiveProfile();

  // Same-origin proxy URL base; forward ?profile and ?video so the API can pick upstream.
  const PROXY_URL_BASE = '/api/video-proxy';
  const proxyQuery = useMemo(() => {
    const qs = new URLSearchParams();
    if (activeProfile) qs.set('profile', String(activeProfile));
    if (qpVideo) qs.set('video', qpVideo);
    return qs.toString();
  }, [activeProfile, qpVideo]);

  // Final image source picks proxy first, then (if proxy fails) tries the direct upstream if safe.
  const IMG_BASE = useDirect
    ? DIRECT_VIDEO_STREAM
    : proxyQuery
    ? `${PROXY_URL_BASE}?${proxyQuery}`
    : PROXY_URL_BASE;

  // Add a cache-buster whenever we retry or status toggles
  const IMG_SRC = useMemo(() => {
    if (!IMG_BASE) return '';
    const sep = IMG_BASE.includes('?') ? '&' : '?';
    return `${IMG_BASE}${sep}b=${buster}`;
  }, [IMG_BASE, buster]);

  // Health probing – prefer same-origin proxy; falls back to direct if proxy not desired.
  const PROXY_HEALTH = proxyQuery ? `/api/video-health?${proxyQuery}` : '/api/video-health';
  const canProbeDirectHealth = DIRECT_VIDEO_HEALTH && !willBeMixedContent(DIRECT_VIDEO_HEALTH);

  // Always try proxy health (same-origin, recommended). If you explicitly don’t want the proxy,
  // swap to `canProbeDirectHealth ? DIRECT_VIDEO_HEALTH : undefined`.
  const HEALTH_URL = PROXY_HEALTH || (canProbeDirectHealth ? DIRECT_VIDEO_HEALTH : undefined);

  const { status: healthStatus, latency } = useHttpStatus(HEALTH_URL, {
    intervalMs: 5000,
    timeoutMs: 2500,
  });

  // Track last time the MJPEG actually loaded successfully; helps override status
  const lastImgOkAt = useRef<number | null>(null);

  // Derive an effective status combining health + image reality + offline
  const effectiveStatus: ServerStatus = useMemo(() => {
    if (isOffline) return 'disconnected';

    // If the image has loaded very recently, consider us connected regardless of transient health
    const now = Date.now();
    const recentOk = lastImgOkAt.current && now - lastImgOkAt.current < 8000; // 8s grace
    if (recentOk) return 'connected';

    // Otherwise rely on health if available (proxy health is preferred)
    if (healthStatus) return healthStatus;

    // No health available: infer from img error state
    return imgError ? 'disconnected' : 'connecting';
  }, [healthStatus, imgError, isOffline]);

  // Notify parent when the effective online state flips
  const wasOnline = useRef<boolean | null>(null);
  useEffect(() => {
    const online = effectiveStatus === 'connected';
    if (wasOnline.current !== online) {
      wasOnline.current = online;
      onVideoStatusChange?.(online);
    }
  }, [effectiveStatus, onVideoStatusChange]);

  // Reset image error whenever we change the src
  useEffect(() => {
    setImgError(false);
  }, [IMG_SRC]);

  // Listen for browser offline/online to improve UX
  useEffect(() => {
    if (typeof window === 'undefined') return;
    const on = () => setIsOffline(false);
    const off = () => setIsOffline(true);
    window.addEventListener('online', on);
    window.addEventListener('offline', off);
    return () => {
      window.removeEventListener('online', on);
      window.removeEventListener('offline', off);
    };
  }, []);

  const onImgLoad = () => {
    lastImgOkAt.current = Date.now();
    // keep current source selection; status will go green
  };

  const onImgError = () => {
    setImgError(true);
    // If proxy failed, try direct (but only if it won't be mixed content)
    if (!useDirect) {
      const safe = DIRECT_VIDEO_STREAM && !willBeMixedContent(DIRECT_VIDEO_STREAM);
      if (safe) {
        setUseDirect(true);
        setBuster(Date.now()); // force immediate retry with direct
      }
    }
  };

  const retry = () => {
    // Prefer proxy again on retry
    setUseDirect(false);
    setImgError(false);
    lastImgOkAt.current = null;
    setBuster(Date.now());
  };

  const showImg = Boolean(IMG_SRC);
  const titleSuffix = latency != null ? ` • ${latency}ms` : '';
  const statusLabel =
    effectiveStatus === 'no_camera' ? 'No camera' : effectiveStatus.charAt(0).toUpperCase() + effectiveStatus.slice(1);

  return (
    <div
      className={`relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden ${className}`}
      style={{ aspectRatio: '16 / 9' }}
    >
      {/* Status pill (top-left) */}
      <div className="absolute top-2 left-2 z-20 flex items-center gap-2 px-2 py-1 bg-black/45 backdrop-blur rounded border border-white/15 text-white text-xs">
        <StatusDot status={effectiveStatus} title={`Video: ${statusLabel}${titleSuffix}`} />
        <span className="font-semibold">Video</span>
        <span className="text-white/80">
          {effectiveStatus === 'no_camera' ? 'No camera' : latency != null ? `${latency}ms` : '—'}
        </span>
        <span className="ml-1 px-1 rounded bg-white/10 border border-white/10" title="Stream source">
          {useDirect ? 'direct' : 'proxy'}
        </span>
        {isOffline && <span className="ml-1 px-1 rounded bg-rose-500/80 text-black">offline</span>}
      </div>

      {/* Live region for screen readers */}
      <div className="sr-only" aria-live="polite">
        Video status: {statusLabel}. {latency != null ? `Latency ${latency} milliseconds.` : ''}
      </div>

      {/* Main media */}
      <div className="absolute inset-0 bg-black">
        {showImg ? (
          <img
            key={IMG_SRC /* force a new request when buster changes */}
            src={IMG_SRC}
            alt="Live video feed"
            className={`w-full h-full ${objectFit === 'cover' ? 'object-cover' : 'object-contain'}`}
            onLoad={onImgLoad}
            onError={onImgError}
            draggable={false}
          />
        ) : (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-2">
              {effectiveStatus === 'no_camera'
                ? 'No camera detected'
                : effectiveStatus === 'connecting'
                ? 'Connecting...'
                : 'Video feed not connected'}
            </div>
            <div className="flex gap-2">
              <button
                type="button"
                className="px-2 py-1 rounded bg-black/40 hover:bg-black/55 border border-white/20 text-xs"
                onClick={retry}
              >
                Retry
              </button>
              {DIRECT_VIDEO_STREAM && (
                <a
                  className="px-2 py-1 rounded bg-black/40 hover:bg-black/55 border border-white/20 text-xs"
                  href={DIRECT_VIDEO_STREAM}
                  target="_blank"
                  rel="noreferrer"
                >
                  Open direct
                </a>
              )}
            </div>
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
