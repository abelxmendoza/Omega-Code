// File: /Omega-Code/ui/robot-controller-ui/src/utils/resolveWsUrl.ts
// Summary:
//   Profile-aware URL resolver for WS endpoints (robust).
//   - Reads NEXT_PUBLIC_NETWORK_PROFILE ('lan' | 'tailscale' | 'local'), with ?profile= override.
//   - Direct URL precedence: ?ws= override → BASE key → BASE_PROFILE keys.
//   - Falls back to host-based envs (NEXT_PUBLIC_ROBOT_HOST_[PROFILE] + defaultPort + path).
//   - Optional same-origin candidate (useful when reverse-proxying from the UI host).
//   - Auto ws:// ↔ wss:// based on page scheme; careful path normalization; respects explicit ports.
//   - Uses runtime window.__ENV__ (optional) in addition to build-time NEXT_PUBLIC_* env.
//
// Env examples:
//   NEXT_PUBLIC_NETWORK_PROFILE=tailscale
//   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE=ws://omega1-tailscale:8081
//   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT=ws://omega1-tailscale:8081     ← base (all profiles) optional
//   NEXT_PUBLIC_ROBOT_HOST_TAILSCALE=omega1-tailscale
//   NEXT_PUBLIC_ROBOT_HOST_LAN=omega1.local
//
// Dev override (highest precedence):
//   https://app.example/?profile=tailscale&ws=ws://omega1-tailscale:8081

'use client';

export type NetProfile = 'lan' | 'tailscale' | 'local';

const DEBUG =
  typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
    (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:resolve]', ...a);

const readEnv = (k: string): string | undefined =>
  // Build-time (Next.js)
  (process.env as any)?.[k] ??
  // Optional runtime injection (window.__ENV__ = { … })
  (typeof window !== 'undefined' ? (window as any).__ENV__?.[k] : undefined);

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

/** Does the host string already contain an explicit port? (IPv4/hostname:port or [IPv6]:port) */
function hasExplicitPort(host: string): boolean {
  if (!host) return false;
  // bracketed IPv6 with port: [2001:db8::1]:8081
  if (/^\[.*\]:\d+$/.test(host)) return true;
  // hostname/IPv4 with port: foo.local:8081 or 192.168.1.10:8081
  if (/^[^[\]/:]+:\d+$/.test(host)) return true;
  // bare IPv6 without brackets isn't URL-safe; treat as no explicit port
  return false;
}

/** Convert a url/host into a ws(s) URL with optional port + path, honoring https→wss and explicit ports. */
function toWs(urlOrHost: string, port?: string, path = ''): string {
  const p = ensureLeadingSlash(path);

  // Already ws(s):// → append path
  if (/^wss?:\/\//i.test(urlOrHost)) return urlOrHost + p;

  // http(s):// → convert to ws(s)://
  if (/^https?:\/\//i.test(urlOrHost)) {
    const u = new URL(urlOrHost);
    u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
    if (port) u.port = port; // override if caller set
    u.pathname = (u.pathname || '/') + p;
    return u.toString();
  }

  // Bare host (maybe includes port like host:1234 or [v6]:1234)
  const scheme = pageWss() ? 'wss' : 'ws';
  const needsPort = port && !hasExplicitPort(urlOrHost);
  const hostPort = needsPort ? `${urlOrHost}:${port}` : urlOrHost;
  return `${scheme}://${hostPort}${p}`;
}

/**
 * Direct URL for baseKey + profile, if provided:
 *   e.g. baseKey='NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT', prof='tailscale'
 *   → reads NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE
 */
function directUrlFor(baseKey: string, prof: NetProfile): string | undefined {
  const key = `${baseKey}_${prof.toUpperCase().replace(/-/g, '_')}`;
  const v = readEnv(key);
  return v && v.trim() ? v.trim() : undefined;
}

/** Base direct URL for baseKey without profile suffix (highest env precedence after ?ws=). */
function directBaseUrl(baseKey: string): string | undefined {
  const v = readEnv(baseKey);
  return v && v.trim() ? v.trim() : undefined;
}

/**
 * Host-based URL for profile when direct URL missing:
 *   NEXT_PUBLIC_ROBOT_HOST_[PROFILE] + defaultPort + path
 */
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
  /** Port to use when building URLs from hosts (e.g., 8081 for movement). */
  defaultPort?: string;
  /** Optional WS path (e.g., '/ws' or '/ultrasonic'). */
  path?: string;
  /** Include a same-origin candidate at the end. Default: true. */
  includeSameOrigin?: boolean;
  /**
   * Allow a dev-time query override ?ws=ws://host:port/path.
   * Default: true (handy for quick testing).
   */
  allowQueryWsOverride?: boolean;
}

/**
 * Returns candidate URLs ordered as:
 *   [ ?ws= override ] →
 *   [ BASE (no profile) ] →
 *   [ active profile: direct → host ] →
 *   [ other profiles: direct → host ] →
 *   [ optional same-origin ]
 *
 * Blanks skipped; duplicates de-duped (first occurrence wins).
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

  // 0) Optional query override (?ws=ws://...)
  const fromQuery = allowQueryWsOverride ? getQueryParam('ws') : undefined;

  // 1) Base direct URL (env without profile suffix)
  const baseDirect = directBaseUrl(baseKey);

  // 2) Active profile candidates (prefer direct, then host)
  const activeDirect = directUrlFor(baseKey, active);
  const activeHost   = hostUrlFor(active, { defaultPort, path });

  // 3) Other profiles (each: direct then host)
  const otherDirects = others.map((p) => directUrlFor(baseKey, p));
  const otherHosts   = others.map((p) => hostUrlFor(p, { defaultPort, path }));

  // 4) Same-origin (often behind reverse proxy)
  const same = includeSameOrigin ? sameOriginUrl({ defaultPort, path }) : undefined;

  // Build, normalize, and de-dupe
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
    .map((u) => toWs(u, '', path)) // toWs is idempotent for ws(s):// and converts http(s)://
    .filter((u) => (seen.has(u) ? false : (seen.add(u), true)));

  dlog('candidates', { active, list });
  return list;
}

/** Return the first/best URL ('' if none). */
export function resolveWsUrl(baseKey: WsKey, opts: ResolveOpts = {}): string {
  return resolveWsCandidates(baseKey, opts)[0] ?? '';
}

/** Optional: return a structured explanation for debugging UIs. */
export function explainWsResolution(baseKey: WsKey, opts: ResolveOpts = {}) {
  const active = getActiveProfile();
  const list = resolveWsCandidates(baseKey, opts);
  return { activeProfile: active, candidates: list };
}
