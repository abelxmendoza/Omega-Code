// File: /Omega-Code/ui/robot-controller-ui/src/utils/resolveWsUrl.ts
// Summary:
//   Profile-aware URL resolver for WS endpoints (browser-safe).
//   - Avoids dynamic process.env access in client bundles by using a static ENV shim.
//   - ?profile and optional ?ws override supported.
//   - Candidate generation with direct/base/host/same-origin + https→wss handling.

'use client';

export type NetProfile = 'lan' | 'tailscale' | 'local';

/** STATIC env shim so Next.js can inline values safely into the client bundle. */
const ENV = {
  // flags
  NEXT_PUBLIC_WS_DEBUG: process.env.NEXT_PUBLIC_WS_DEBUG,
  NEXT_PUBLIC_NETWORK_PROFILE: process.env.NEXT_PUBLIC_NETWORK_PROFILE,

  // robot host fallbacks (optional)
  NEXT_PUBLIC_ROBOT_HOST_TAILSCALE: process.env.NEXT_PUBLIC_ROBOT_HOST_TAILSCALE,
  NEXT_PUBLIC_ROBOT_HOST_LAN: process.env.NEXT_PUBLIC_ROBOT_HOST_LAN,
  NEXT_PUBLIC_ROBOT_HOST_LOCAL: process.env.NEXT_PUBLIC_ROBOT_HOST_LOCAL,

  // movement
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT: process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT,
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE: process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN: process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL: process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL,

  // ultrasonic
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC: process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC,
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE: process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN: process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL: process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL,

  // line-tracker
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER,
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL,

  // lighting
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING,
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL,

  // location
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION,
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_TAILSCALE: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LAN: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LOCAL: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LOCAL,

  // video (used indirectly by VideoFeed; kept here for completeness)
  NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
  NEXT_PUBLIC_VIDEO_STREAM_URL_LAN: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
  NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
} as const;

type EnvKey = keyof typeof ENV;

const readEnv = (k: string): string | undefined => {
  // build-time inlined values
  if ((ENV as Record<string, string | undefined>)[k as EnvKey] != null) {
    return (ENV as Record<string, string | undefined>)[k as EnvKey];
  }
  // optional runtime injection window.__ENV__ = { KEY: "value" }
  if (typeof window !== 'undefined' && (window as any).__ENV__) {
    const v = (window as any).__ENV__[k];
    if (typeof v === 'string' && v.length) return v;
  }
  return undefined;
};

const DEBUG =
  (readEnv('NEXT_PUBLIC_WS_DEBUG') === '1') ||
  (typeof window !== 'undefined' && (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:resolve]', ...a);

function getQueryParam(name: string): string | undefined {
  if (typeof window === 'undefined') return undefined;
  const v = new URLSearchParams(window.location.search).get(name);
  return v?.trim() || undefined;
}

function getUrlProfileOverride(): NetProfile | null {
  const p = getQueryParam('profile')?.toLowerCase();
  return p === 'lan' || p === 'tailscale' || p === 'local' ? (p as NetProfile) : null;
}

export function getActiveProfile(): NetProfile {
  const fromUrl = getUrlProfileOverride();
  if (fromUrl) return fromUrl;
  const raw = (readEnv('NEXT_PUBLIC_NETWORK_PROFILE') || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as NetProfile[]).includes(raw as NetProfile)
    ? (raw as NetProfile)
    : 'local';
}

function pageWss(): boolean {
  return typeof window !== 'undefined' && window.location.protocol === 'https:';
}

function ensureLeadingSlash(path: string): string {
  if (!path) return '';
  return path.startsWith('/') ? path : '/' + path;
}

function hasExplicitPort(host: string): boolean {
  if (!host) return false;
  if (/^\[.*\]:\d+$/.test(host)) return true;       // [v6]:port
  if (/^[^[\]/:]+:\d+$/.test(host)) return true;    // host:port or ipv4:port
  return false;
}

function toWs(urlOrHost: string, port?: string, path = ''): string {
  const p = ensureLeadingSlash(path);

  if (/^wss?:\/\//i.test(urlOrHost)) return urlOrHost + p;

  if (/^https?:\/\//i.test(urlOrHost)) {
    const u = new URL(urlOrHost);
    u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
    if (port) u.port = port;
    u.pathname = (u.pathname || '/') + p;
    return u.toString();
  }

  const scheme = pageWss() ? 'wss' : 'ws';
  const needsPort = port && !hasExplicitPort(urlOrHost);
  const hostPort = needsPort ? `${urlOrHost}:${port}` : urlOrHost;
  return `${scheme}://${hostPort}${p}`;
}

function directUrlFor(baseKey: string, prof: NetProfile): string | undefined {
  const key = `${baseKey}_${prof.toUpperCase().replace(/-/g, '_')}`;
  const v = readEnv(key);
  return v && v.trim() ? v.trim() : undefined;
}

function directBaseUrl(baseKey: string): string | undefined {
  const v = readEnv(baseKey);
  return v && v.trim() ? v.trim() : undefined;
}

function hostUrlFor(
  prof: NetProfile,
  opts: { defaultPort: string; path?: string }
): string | undefined {
  const key = `NEXT_PUBLIC_ROBOT_HOST_${prof.toUpperCase()}`;
  const host = (readEnv(key) || '').trim();
  if (!host) return undefined;
  return toWs(host, opts.defaultPort, opts.path ?? '');
}

function sameOriginUrl(opts: { defaultPort: string; path?: string }): string | undefined {
  if (typeof window === 'undefined') return undefined;
  const host = window.location.hostname;
  if (!host) return undefined;
  return toWs(host, opts.defaultPort, opts.path ?? '');
}

export type WsKey =
  | 'NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LOCATION'
  | 'NEXT_PUBLIC_VIDEO_STREAM_URL';

export interface ResolveOpts {
  defaultPort?: string;
  path?: string;
  includeSameOrigin?: boolean;
  allowQueryWsOverride?: boolean;
}

export function resolveWsCandidates(
  baseKey: WsKey,
  opts: ResolveOpts = {}
): string[] {
  const {
    defaultPort = '8081',
    path = '',
    includeSameOrigin = true,
    allowQueryWsOverride = true,
  } = opts;

  const active = getActiveProfile();
  const others: NetProfile[] =
    active === 'tailscale' ? ['lan', 'local'] :
    active === 'lan'       ? ['tailscale', 'local'] :
                              ['lan', 'tailscale'];

  const fromQuery = allowQueryWsOverride ? getQueryParam('ws') : undefined;

  const baseDirect   = directBaseUrl(baseKey);
  const activeDirect = directUrlFor(baseKey, active);
  const activeHost   = hostUrlFor(active, { defaultPort, path });
  const otherDirects = others.map((p) => directUrlFor(baseKey, p));
  const otherHosts   = others.map((p) => hostUrlFor(p, { defaultPort, path }));
  const same         = includeSameOrigin ? sameOriginUrl({ defaultPort, path }) : undefined;

  const seen = new Set<string>();
  const raw = [
    fromQuery,
    baseDirect,
    activeDirect,
    activeHost,
    ...otherDirects,
    ...otherHosts,
    same,
  ].filter(Boolean) as string[];

  const list = raw
    .map((u) => toWs(u, '', path))
    .filter((u) => (seen.has(u) ? false : (seen.add(u), true)));

  dlog('candidates', { active, list });
  return list;
}

export function resolveWsUrl(baseKey: WsKey, opts: ResolveOpts = {}): string {
  return resolveWsCandidates(baseKey, opts)[0] ?? '';
}

export function explainWsResolution(baseKey: WsKey, opts: ResolveOpts = {}) {
  const active = getActiveProfile();
  const candidates = resolveWsCandidates(baseKey, opts);
  return { activeProfile: active, candidates };
}
