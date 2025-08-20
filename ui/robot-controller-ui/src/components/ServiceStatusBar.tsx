/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/ServiceStatusBar.tsx
# Summary:
#   Compact, live status bar for backend services with a collapsible panel.
#     - Movement   (WebSocket; JSON heartbeat)
#     - Ultrasonic (WebSocket; streaming JSON, no pong)
#     - Line       (WebSocket; JSON heartbeat)
#     - Lighting   (WebSocket; JSON heartbeat)
#     - Video      (HTTP; probes `/api/video-health` by default to avoid mixed-content)
#
#   Improvements:
#   - Collapsible UI (persists state via localStorage)
#   - Uses resolveWsUrl() + required subpaths (consistent with app resolver)
#   - Mixed-content guard for direct /health probes; proxy health preferred
#   - Honors page overrides ?profile=lan|tailscale|local and ?video=<custom-url>
#   - Accessible: aria-expanded, aria-controls, focus ring
*/

'use client';

import React, { useEffect, useMemo, useState } from 'react';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';
import { resolveWsUrl } from '@/utils/resolveWsUrl';

const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG;

/* ----------------------------- helpers ------------------------------ */

/** Read a query param from the current page (client only). */
const getQueryParam = (name: string): string | undefined => {
  if (typeof window === 'undefined') return undefined;
  const v = new URLSearchParams(window.location.search).get(name);
  return v?.trim() || undefined;
};

/** Active UI profile: lan | tailscale | local (honor ?profile override). */
const getActiveProfile = (): 'lan' | 'tailscale' | 'local' => {
  const qp = (getQueryParam('profile') || '').toLowerCase();
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'].includes(env) ? env : 'local') as any;
};

/** Convert direct .../video_feed → .../health. */
const toHealthUrl = (videoUrl?: string) => {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
};

/** Mixed-content check (block http:// off-origin when page is https://). */
const willBeMixedContent = (url?: string): boolean => {
  if (!url || typeof window === 'undefined') return false;
  if (window.location.protocol !== 'https:') return false;
  try {
    const u = new URL(url, window.location.href);
    return u.protocol === 'http:' && u.hostname !== window.location.hostname;
  } catch {
    return false;
  }
};

/** If a resolved WS URL is missing a required subpath (because of host fallback), append it once. */
const ensurePath = (u: string, path: string) =>
  u && !u.includes(path) ? `${u.replace(/\/$/, '')}${path}` : u;

/* ----------------------- WS endpoints via resolver ------------------ */

const MOVE  = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT',     { defaultPort: '8081' });
const ULTRA = ensurePath(resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', { defaultPort: '8080' }), '/ultrasonic');
const LINE  = ensurePath(resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER', { defaultPort: '8090' }), '/line-tracker');
const LIGHT = ensurePath(resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING', { defaultPort: '8082' }), '/lighting');

/* --------------------------- Video endpoints ------------------------ */
/*
  Preferred: same-origin proxy HEALTH endpoint → /api/video-health?profile=<p>[&video=...]
  This avoids mixed-content/CORS and matches your /api/video-proxy usage elsewhere.
  We still compute a direct health URL for fallback/diagnostics when it's safe.
*/

const VIDEO_LAN       = process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN;
const VIDEO_TAILSCALE = process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE;
const VIDEO_LOCAL     = process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL;

const pickDirectVideoByProfile = (): string => {
  const prof = getActiveProfile();
  const qpVideo = getQueryParam('video'); // page override wins
  if (qpVideo) return qpVideo;

  return prof === 'lan'
    ? (VIDEO_LAN || VIDEO_LOCAL || '')
    : prof === 'tailscale'
    ? (VIDEO_TAILSCALE || VIDEO_LOCAL || '')
    : (VIDEO_LOCAL || VIDEO_LAN || VIDEO_TAILSCALE || '');
};

/* ------------------------------ UI bits ----------------------------- */

const Dot: React.FC<{ state: ServiceStatus | HttpStatus }> = ({ state }) => {
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
}> = ({ label, state, latency }) => (
  <div
    className="flex items-center gap-2 text-xs text-white/90"
    title={`${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`}
  >
    <Dot state={state} />
    <span className="font-medium">{label}</span>
    <span className="text-white/60">{state}</span>
    {latency != null && <span className="text-white/50">• {latency} ms</span>}
  </div>
);

// Generic chevron icon (no extra deps)
const Chevron: React.FC<{ open: boolean }> = ({ open }) => (
  <svg
    className={`w-4 h-4 transition-transform ${open ? 'rotate-180' : ''}`}
    viewBox="0 0 20 20"
    fill="currentColor"
    aria-hidden
  >
    <path d="M5.23 7.21a.75.75 0 011.06.02L10 10.17l3.71-2.94a.75.75 0 111.04 1.08l-4.24 3.36a.75.75 0 01-.94 0L5.21 8.31a.75.75 0 01.02-1.1z" />
  </svg>
);

/* ----------------------------- Component ---------------------------- */

const ServiceStatusBar: React.FC = () => {
  const [collapsed, setCollapsed] = useState<boolean>(false);

  // Restore persisted collapsed state
  useEffect(() => {
    try {
      const v = localStorage.getItem('serviceStatusBarCollapsed');
      if (v === '1') setCollapsed(true);
    } catch {}
  }, []);
  // Persist on change
  useEffect(() => {
    try {
      localStorage.setItem('serviceStatusBarCollapsed', collapsed ? '1' : '0');
    } catch {}
  }, [collapsed]);

  // Build proxy health URL (preferred): /api/video-health?profile=<p>[&video=...]
  const VIDEO_PROXY_HEALTH = useMemo(() => {
    const qs = new URLSearchParams();
    qs.set('profile', getActiveProfile());
    const qpVideo = getQueryParam('video');
    if (qpVideo) qs.set('video', qpVideo);
    const url = `/api/video-health?${qs.toString()}`;
    if (DEBUG) console.log('[status] video proxy health:', url);
    return url;
  }, []);

  // Also compute a direct health URL (for fallback only, if safe)
  const VIDEO_DIRECT_HEALTH = useMemo(() => {
    const directVideo = pickDirectVideoByProfile();
    const h = toHealthUrl(directVideo);
    if (DEBUG) console.log('[status] video direct health:', h || '(none)');
    return h;
  }, []);

  // Choose health endpoint: prefer proxy; if unavailable for some reason, use direct when it will not be mixed-content.
  const canUseProxyHealth = !!VIDEO_PROXY_HEALTH; // same-origin → always safe
  const canUseDirectHealth = VIDEO_DIRECT_HEALTH && !willBeMixedContent(VIDEO_DIRECT_HEALTH);

  const healthUrl = canUseProxyHealth
    ? VIDEO_PROXY_HEALTH
    : (canUseDirectHealth ? (VIDEO_DIRECT_HEALTH as string) : '');

  // Video HTTP status polling
  const video: { status: HttpStatus; latency: number | null } =
    healthUrl
      ? useHttpStatus(healthUrl, { intervalMs: 5000, timeoutMs: 2500, treat503AsConnecting: true })
      : { status: 'connecting', latency: null };

  // WS with pong → latency
  const move  = MOVE  ? useWsStatus(MOVE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 }) : { status: 'disconnected' as ServiceStatus, latency: null };
  const line  = LINE  ? useWsStatus(LINE,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 }) : { status: 'disconnected' as ServiceStatus, latency: null };
  const light = LIGHT ? useWsStatus(LIGHT, { pingIntervalMs: 5000, pongTimeoutMs: 2500 }) : { status: 'disconnected' as ServiceStatus, latency: null };

  // WS streaming (any message alive)
  const ultra = ULTRA ? useWsStatus(ULTRA, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 }) : { status: 'disconnected' as ServiceStatus, latency: null };

  const states = [move.status, ultra.status, line.status, light.status, video.status] as const;
  const up = states.filter(s => s === 'connected').length;
  const allUp = up === states.length;
  const anyUp = up > 0;

  // Summary dot color for collapsed header
  const summaryState: ServiceStatus | HttpStatus = allUp ? 'connected' : anyUp ? 'connecting' : 'disconnected';

  return (
    <div className="text-white">
      {/* Collapsed header / toggle */}
      <button
        type="button"
        aria-expanded={!collapsed}
        aria-controls="service-status-panel"
        onClick={() => setCollapsed(v => !v)}
        className="w-full flex items-center justify-between gap-3 px-3 py-2 bg-gray-800/70 hover:bg-gray-800 border border-white/10 rounded-md transition focus:outline-none focus:ring-2 focus:ring-amber-400"
      >
        <div className="flex items-center gap-3 text-sm">
          <Dot state={summaryState} />
          <span className="font-semibold">Services</span>
          <span className="text-white/70">• {up}/{states.length} online</span>
        </div>
        <div className="flex items-center gap-2 text-xs text-white/70">
          <span>{collapsed ? 'Show' : 'Hide'}</span>
          <Chevron open={!collapsed} />
        </div>
      </button>

      {/* Collapsible panel */}
      <div
        id="service-status-panel"
        className={`overflow-hidden transition-all duration-300 ease-out ${collapsed ? 'max-h-0 opacity-0 scale-[0.98]' : 'max-h-40 opacity-100 scale-100'}`}
      >
        <div className="mt-2 px-3 py-2 bg-black/40 border border-white/10 rounded-md">
          <div className="text-xs text-white/70 mb-1" aria-live="polite">
            Detailed status
          </div>
          <div className="flex flex-wrap items-center gap-4">
            <Item label="Movement"    state={move.status}   latency={move.latency} />
            <Item label="Ultrasonic"  state={ultra.status}  />
            <Item label="Line"        state={line.status}   latency={line.latency} />
            <Item label="Lighting"    state={light.status}  latency={light.latency} />
            <Item label="Video"       state={video.status}  latency={video.latency} />
          </div>
        </div>
      </div>
    </div>
  );
};

export default ServiceStatusBar;
