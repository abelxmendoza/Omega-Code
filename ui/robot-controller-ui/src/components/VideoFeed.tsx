/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/VideoFeed.tsx
# Summary:
#   Live MJPEG preview with a mini GPS map, robust status handling, and
#   automatic profile failover (tailscale → lan → local) when sources fail.
#
#   Behavior:
#   - Prefer same-origin proxy: GET /api/video-proxy?profile=<p>[&video=...]
#   - Health via same-origin:   GET /api/video-health?profile=<p>[&video=...]
#   - If the proxy image errors, try direct upstream (only if no mixed content).
#   - If direct also fails, rotate to the next profile (unless ?profile pinned).
#   - Honors page overrides:
#       • ?video=<override-url>  → no rotation; use that upstream (proxy first)
#       • ?profile=lan|tailscale|local → pin to that profile (no rotation)
#
#   Env inputs:
#     NEXT_PUBLIC_NETWORK_PROFILE = lan | tailscale | local
#     NEXT_PUBLIC_VIDEO_STREAM_URL_LAN
#     NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE
#     NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
#
#   Parent callback:
#     onVideoStatusChange(ok: boolean)
*/

'use client';

import React, { useEffect, useMemo, useRef, useState } from 'react';
import GpsLocation from './GpsLocation';
import { useHttpStatus } from '../hooks/useHttpStatus';
import { getActiveProfile } from '@/utils/resolveWsUrl';

type Profile = 'lan' | 'tailscale' | 'local';
type ServerStatus = 'connected' | 'connecting' | 'disconnected' | 'no_camera';

/* ----------------------------- helpers ------------------------------ */

function getQueryParam(name: string): string | undefined {
  if (typeof window === 'undefined') return undefined;
  const v = new URLSearchParams(window.location.search).get(name);
  return v?.trim() || undefined;
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

/** Convert .../video_feed[?...] → sibling /health. */
function toHealthUrl(videoUrl?: string) {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
}

/** Build env-driven direct URLs by profile. */
function envDirectByProfile(): Record<Profile, string | undefined> {
  return {
    lan: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    local: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };
}

/** The rotation order: active first, then the other two (unique). */
function getProfileRotation(active: Profile): Profile[] {
  const all: Profile[] = ['tailscale', 'lan', 'local'];
  return [active, ...all.filter((p) => p !== active)];
}

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
  const qpProfile = (getQueryParam('profile') as Profile | undefined)?.toLowerCase() as Profile | undefined;
  const qpVideo = getQueryParam('video'); // explicit upstream override
  const initialActive = (qpProfile || (getActiveProfile() as Profile)) as Profile;

  // Rotation order (active first). If qpProfile is set, we pin and don't rotate.
  const rotation = useMemo(() => getProfileRotation(initialActive), [initialActive]);
  const rotationEnabled = !qpProfile && !qpVideo; // explicit page overrides disable rotation

  const [mapExpanded, setMapExpanded] = useState(false);
  const [imgError, setImgError] = useState(false);
  const [useDirect, setUseDirect] = useState(false);
  const [buster, setBuster] = useState<number>(() => Date.now());
  const [isOffline, setIsOffline] = useState<boolean>(typeof navigator !== 'undefined' ? !navigator.onLine : false);
  const [profileIdx, setProfileIdx] = useState<number>(0); // index into rotation array
  const attemptsRef = useRef<number>(0); // guard against infinite loops

  const currentProfile = rotation[profileIdx] || initialActive;
  const directByProfile = useMemo(() => envDirectByProfile(), []);

  // If ?video= is present, always use that upstream for direct (and pass to proxy)
  const directForProfile = useMemo(() => {
    if (qpVideo) return qpVideo;
    return directByProfile[currentProfile] || '';
  }, [qpVideo, directByProfile, currentProfile]);

  const directHealthForProfile = useMemo(() => toHealthUrl(directForProfile), [directForProfile]);

  // Same-origin proxy base, always forward current profile and optional ?video
  const PROXY_URL_BASE = '/api/video-proxy';
  const PROXY_HEALTH_BASE = '/api/video-health';
  const proxyQuery = useMemo(() => {
    const qs = new URLSearchParams();
    qs.set('profile', String(currentProfile));
    if (qpVideo) qs.set('video', qpVideo);
    return qs.toString();
  }, [currentProfile, qpVideo]);

  // Final image source: proxy first, then direct (safe only)
  const IMG_BASE = useDirect
    ? directForProfile
    : proxyQuery
    ? `${PROXY_URL_BASE}?${proxyQuery}`
    : PROXY_URL_BASE;

  const IMG_SRC = useMemo(() => {
    if (!IMG_BASE) return '';
    const sep = IMG_BASE.includes('?') ? '&' : '?';
    return `${IMG_BASE}${sep}b=${buster}`;
  }, [IMG_BASE, buster]);

  // Health: prefer proxy health. If for some reason that's disabled, allow safe direct.
  const proxyHealth = `${PROXY_HEALTH_BASE}?${proxyQuery}`;
  const canProbeDirectHealth = directHealthForProfile && !willBeMixedContent(directHealthForProfile);
  const HEALTH_URL = proxyHealth || (canProbeDirectHealth ? directHealthForProfile : undefined);

  const { status: healthStatus, latency } = useHttpStatus(HEALTH_URL, {
    intervalMs: 5000,
    timeoutMs: 2500,
  });

  // Track last successful image load
  const lastImgOkAt = useRef<number | null>(null);

  // Effective status derives from connectivity + health + recent img load
  const effectiveStatus: ServerStatus = useMemo(() => {
    if (isOffline) return 'disconnected';
    const now = Date.now();
    const recentOk = lastImgOkAt.current && now - lastImgOkAt.current < 8000;
    if (recentOk) return 'connected';
    if (healthStatus) return healthStatus;
    return imgError ? 'disconnected' : 'connecting';
  }, [healthStatus, imgError, isOffline]);

  // Notify parent when online state flips
  const wasOnline = useRef<boolean | null>(null);
  useEffect(() => {
    const online = effectiveStatus === 'connected';
    if (wasOnline.current !== online) {
      wasOnline.current = online;
      onVideoStatusChange?.(online);
    }
  }, [effectiveStatus, onVideoStatusChange]);

  // Reset image error on src changes
  useEffect(() => {
    setImgError(false);
  }, [IMG_SRC]);

  // Listen for offline/online
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

  const rotateProfile = () => {
    if (!rotationEnabled) return false;
    if (attemptsRef.current >= rotation.length) return false; // tried them all
    const next = (profileIdx + 1) % rotation.length;
    if (next === profileIdx) return false;
    setProfileIdx(next);
    attemptsRef.current += 1;
    setUseDirect(false); // when switching profile, always try proxy again first
    setBuster(Date.now()); // force reload
    return true;
  };

  const onImgLoad = () => {
    lastImgOkAt.current = Date.now();
    attemptsRef.current = 0; // success → reset fail counter
  };

  const onImgError = () => {
    setImgError(true);

    // Step 1: if we were on proxy, try direct (but only if safe)
    if (!useDirect) {
      const safe = directForProfile && !willBeMixedContent(directForProfile);
      if (safe) {
        setUseDirect(true);
        setBuster(Date.now());
        return;
      }
    }

    // Step 2: rotate to next profile (proxy first), unless pinned
    if (rotateProfile()) return;

    // Step 3: if nothing else to try, just bump buster to retry current source
    setBuster(Date.now());
  };

  const retry = () => {
    // Manual retry: prefer proxy of current profile again
    attemptsRef.current = 0;
    setUseDirect(false);
    setImgError(false);
    lastImgOkAt.current = null;
    setBuster(Date.now());
  };

  const showImg = Boolean(IMG_SRC);
  const titleSuffix = latency != null ? ` • ${latency}ms` : '';
  const statusLabel =
    effectiveStatus === 'no_camera'
      ? 'No camera'
      : effectiveStatus.charAt(0).toUpperCase() + effectiveStatus.slice(1);

  // Pretty badge for current stream provenance
  const sourceBadge = `${useDirect ? 'direct' : 'proxy'} · ${qpVideo ? 'custom' : currentProfile}`;

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
        <span
          className="ml-1 px-1 rounded bg-white/10 border border-white/10"
          title={`Stream source: ${sourceBadge}`}
        >
          {sourceBadge}
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
              {directForProfile && (
                <a
                  className="px-2 py-1 rounded bg-black/40 hover:bg-black/55 border border-white/20 text-xs"
                  href={directForProfile}
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
