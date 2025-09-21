/*
# File: /Omega-Code/ui/robot-controller-ui/src/utils/connectLightingWs.ts
# Summary:
#   Lighting WS helper with graceful fallback + path de-dup.
#   - lightingWsUrl(): best single URL for the active profile
#   - lightingCandidates(): ordered fallback list (profile → others → same-origin)
#   - connectLightingWs(opts): tries candidates until one connects; supports AbortSignal
#   - openLightingSocket(): direct opener (no fallback), auto-upgrades ws:// → wss://
#
# Defaults here match your env: port 8082, path /lighting.
# Enable debug logs by setting NEXT_PUBLIC_WS_DEBUG=1
*/

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

const DEFAULT_PORT = '8082';
const DEFAULT_PATH = '/lighting';

const DEBUG =
  typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
    (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:lighting]', ...a);

/** Ensure the lighting path appears exactly once (avoids ".../lighting/lighting"). */
function ensurePathOnce(u: string, path = DEFAULT_PATH): string {
  if (!u) return u;
  const want = path.startsWith('/') ? path : `/${path}`;
  // normalize trailing slash
  const base = u.replace(/\/+$/, '');
  return base.endsWith(want) ? base : `${base}${want}`;
}

/** Simple de-dupe preserving first occurrence. */
function dedupe(list: string[]): string[] {
  const seen = new Set<string>();
  return list.filter((u) => (u && !seen.has(u) ? (seen.add(u), true) : false));
}

export interface LightingConnectOpts {
  /** Per-attempt timeout (ms). Default 6000. */
  timeoutMs?: number;
  /** Abort all remaining attempts (e.g., on component unmount). */
  signal?: AbortSignal;
  /** Called before each attempt with the normalized URL. */
  onAttempt?: (url: string, index: number, total: number) => void;
  /** Delay between attempts (ms). Default in wsConnect. */
  backoffMs?: number;
  /** Optional ws path; default '/lighting'. */
  path?: string;
  /** Override default port; default 8082. */
  port?: string;
}

/** Best single WS URL for lighting (active profile). */
export function lightingWsUrl(opts?: { path?: string; port?: string }): string {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;

  // IMPORTANT: don't pass `path` into the resolver; we normalize ourselves to avoid double-append.
  const raw = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING', {
    defaultPort: port,
    includeSameOrigin: true,
  });

  const url = raw ? ensurePathOnce(raw, path) : '';
  dlog('resolved URL', url);
  return url;
}

/** Ordered candidates (active profile → others → same-origin). */
export function lightingCandidates(opts?: { path?: string; port?: string }): string[] {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;

  // IMPORTANT: don't pass `path` here; normalize ourselves.
  const raw = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING', {
    defaultPort: port,
    includeSameOrigin: true,
  });

  // normalize & de-dupe
  const list = dedupe(raw.map((u) => ensurePathOnce(u, path)));
  dlog('candidates', list);
  return list;
}

/** Connect with fallback; returns an OPEN WebSocket or throws a descriptive Error. */
export async function connectLightingWs(
  opts: LightingConnectOpts = {}
): Promise<WebSocket> {
  const {
    timeoutMs = 6000,
    signal,
    onAttempt,
    backoffMs,
    path = DEFAULT_PATH,
    port = DEFAULT_PORT,
  } = opts;

  const candidates = lightingCandidates({ path, port });
  if (!candidates.length) {
    throw new Error('Lighting WS: no candidates were generated (check env).');
  }

  try {
    const { ws, url } = await connectWithFallback(candidates, timeoutMs, {
      signal,
      onAttempt,
      backoffMs,
    });
    dlog('connected via', url);
    (ws as any)._chosenUrl = url; // handy for debugging
    return ws;
  } catch (err: any) {
    const tried = candidates.join(', ');
    const msg = err?.message ? String(err.message) : 'unknown error';
    const e = new Error(
      `Lighting WS: all candidates failed. Tried: ${tried}. Last error: ${msg}`
    );
    (e as any).cause = err;
    throw e;
  }
}

/** Direct open (no fallback). Returns null if URL missing or constructor throws. */
export function openLightingSocket(opts?: { path?: string; port?: string }): WebSocket | null {
  const url = lightingWsUrl(opts);
  if (!url) return null;
  try {
    const ws = new WebSocket(upgradeWsForHttps(url));
    (ws as any)._chosenUrl = url;
    return ws;
  } catch {
    return null;
  }
}
