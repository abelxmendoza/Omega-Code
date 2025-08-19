/*
# File: /Omega-Code/ui/robot-controller-ui/src/utils/resolveWsUrl.ts
# Summary:
#   Profile-aware URL resolver for WebSocket endpoints (browser-safe & path-safe).
#
#   Highlights:
#   - Static ENV shim so Next.js can inline at build time (no dynamic process.env reads).
#   - ?profile override (lan|tailscale|local) and optional ?ws=ws://host:port/path override.
#   - Candidate generation: query override → base direct → active (direct→host) → others → same-origin.
#   - Smart ws:// ↔ wss://: honors page scheme, but NEXT_PUBLIC_WS_FORCE_INSECURE=1 forces ws://.
#   - **Path-safe**: only applies `opts.path` if the URL’s current path is empty or "/"
#     (prevents “…/lighting/lighting” when env already includes a path).
*/

'use client';

export type NetProfile = 'lan' | 'tailscale' | 'local';

/** ---------- Static ENV shim (inlined by Next.js) ---------- */
const ENV = {
  // flags
  NEXT_PUBLIC_WS_DEBUG:               process.env.NEXT_PUBLIC_WS_DEBUG,
  NEXT_PUBLIC_WS_FORCE_INSECURE:      process.env.NEXT_PUBLIC_WS_FORCE_INSECURE,
  NEXT_PUBLIC_NETWORK_PROFILE:        process.env.NEXT_PUBLIC_NETWORK_PROFILE,

  // optional robot host fallbacks
  NEXT_PUBLIC_ROBOT_HOST_TAILSCALE:   process.env.NEXT_PUBLIC_ROBOT_HOST_TAILSCALE,
  NEXT_PUBLIC_ROBOT_HOST_LAN:         process.env.NEXT_PUBLIC_ROBOT_HOST_LAN,
  NEXT_PUBLIC_ROBOT_HOST_LOCAL:       process.env.NEXT_PUBLIC_ROBOT_HOST_LOCAL,

  // movement
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT:               process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT,
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE:     process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN:           process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL:         process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL,

  // ultrasonic
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC:             process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC,
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE:   process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN:         process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL:       process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL,

  // line-tracker
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER:           process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER,
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE: process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN:       process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL:     process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL,

  // lighting
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING:               process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING,
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE:     process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN:           process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL:         process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL,

  // location
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION:               process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION,
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_TAILSCALE:     process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_TAILSCALE,
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LAN:           process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LAN,
  NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LOCAL:         process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LOCAL,

  // video (kept here for completeness)
  NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE:            process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
  NEXT_PUBLIC_VIDEO_STREAM_URL_LAN:                  process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
  NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL:                process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
} as const;

type EnvKey = keyof typeof ENV;

const readEnv = (k: string): string | undefined => {
  // 1) build-time inline
  const val = (ENV as Record<string, string | undefined>)[k as EnvKey];
  if (val != null && val !== '') return val;
  // 2) optional runtime override window.__ENV__ = { KEY: "value" }
  if (typeof window !== 'undefined' && (window as any).__ENV__) {
    const v = (window as any).__ENV__[k];
    if (typeof v === 'string' && v.length) return v;
  }
  return undefined;
};

const DEBUG =
  readEnv('NEXT_PUBLIC_WS_DEBUG') === '1' ||
  (typeof window !== 'undefined' && (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:resolve]', ...a);

/** ---------- Small helpers ---------- */
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

function shouldUseWss(): boolean {
  // Force insecure if requested
  if (readEnv('NEXT_PUBLIC_WS_FORCE_INSECURE') === '1') return false;
  return typeof window !== 'undefined' && window.location.protocol === 'https:';
}

function ensureLeadingSlash(path: string): string {
  if (!path) return '';
  return path.startsWith('/') ? path : '/' + path;
}

/** Does host contain an explicit port? (hostname:8081 or [v6]:8081) */
function hasExplicitPort(host: string): boolean {
  if (!host) return false;
  if (/^\[.*\]:\d+$/.test(host)) return true;        // [v6]:port
  if (/^[^[\]/:]+:\d+$/.test(host)) return true;     // host:port or ipv4:port
  return false;
}

/**
 * Convert a url/host into a ws(s) URL with optional port + path.
 * - Respects explicit ws(s)/http(s) schemes.
 * - **Path-safe**: only applies `path` if the URL’s path is empty or "/".
 * - Bare hosts pick scheme based on page (wss if https) unless forced insecure.
 */
function toWs(urlOrHost: string, port?: string, path = ''): string {
  const p = ensureLeadingSlash(path);

  // ws(s):// → preserve existing path; only apply port/path if path is empty/root
  if (/^wss?:\/\//i.test(urlOrHost)) {
    const u = new URL(urlOrHost);
    if (port) u.port = port; // allow override
    const hasMeaningfulPath = u.pathname && u.pathname !== '/' && u.pathname !== '';
    if (!hasMeaningfulPath && p) u.pathname = p;
    return u.toString();
  }

  // http(s):// → convert to ws(s)://
  if (/^https?:\/\//i.test(urlOrHost)) {
    const u = new URL(urlOrHost);
    u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
    if (port) u.port = port;
    const hasMeaningfulPath = u.pathname && u.pathname !== '/' && u.pathname !== '';
    if (!hasMeaningfulPath && p) u.pathname = p;
    return u.toString();
  }

  // bare host → build scheme://host[:port]/path
  const scheme = shouldUseWss() ? 'wss' : 'ws';
  const needsPort = port && !hasExplicitPort(urlOrHost);
  const hostPort = needsPort ? `${urlOrHost}:${port}` : urlOrHost;
  return `${scheme}://${hostPort}${p}`;
}

/** ---------- ENV lookups ---------- */
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

/** ---------- Public API ---------- */
export type WsKey =
  | 'NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LOCATION'
  | 'NEXT_PUBLIC_VIDEO_STREAM_URL';

export interface ResolveOpts {
  /** Port to use when constructing from host vars. Default 8081. */
  defaultPort?: string;
  /** Suggested WS path (e.g. '/lighting'). Only applied if current path is empty or '/'. */
  path?: string;
  /** Include same-origin candidate at the end. Default true. */
  includeSameOrigin?: boolean;
  /** Allow ?ws=ws://host:port/path query override. Default true. */
  allowQueryWsOverride?: boolean;
}

/**
 * Ordered candidates:
 *   [ ?ws= ] → [ base direct ] → [ active: direct → host ] → [ others ] → [ same-origin ]
 * Blanks removed; duplicates de-duped. Paths are handled safely.
 */
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

  const fromQuery   = allowQueryWsOverride ? getQueryParam('ws') : undefined;
  const baseDirect  = directBaseUrl(baseKey);
  const activeDirect= directUrlFor(baseKey, active);
  const activeHost  = hostUrlFor(active, { defaultPort, path });

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

  // Normalize to ws(s) and apply path only where appropriate
  const list = raw
    .map((u) => toWs(u, '', path))
    .filter((u) => (seen.has(u) ? false : (seen.add(u), true)));

  dlog('candidates', { active, list });
  return list;
}

/** First/best candidate ('' if none). */
export function resolveWsUrl(baseKey: WsKey, opts: ResolveOpts = {}): string {
  return resolveWsCandidates(baseKey, opts)[0] ?? '';
}

/** Debug helper for UIs. */
export function explainWsResolution(baseKey: WsKey, opts: ResolveOpts = {}) {
  const activeProfile = getActiveProfile();
  const candidates = resolveWsCandidates(baseKey, opts);
  return { activeProfile, candidates };
}
