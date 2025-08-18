// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectLocationWs.ts
// Summary:
//   Location WS helper with graceful fallback across profiles.
//   - locationWsUrl(): best single URL for the active profile
//   - locationCandidates(): ordered fallback list (profile → others → same-origin)
//   - connectLocationWs(opts): tries candidates until one connects; supports AbortSignal
//   - openLocationSocket(): direct opener (no fallback), auto-upgrades ws:// → wss://
//
// Defaults match your env: port 8091, path /location.
// Enable debug logs by setting NEXT_PUBLIC_WS_DEBUG=1

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

const DEFAULT_PORT = '8091';
const DEFAULT_PATH = '/location';

const DEBUG =
  typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
    (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:location]', ...a);

export interface LocationConnectOpts {
  /** Per-attempt timeout (ms). Default 6000. */
  timeoutMs?: number;
  /** Abort all remaining attempts (e.g., on component unmount). */
  signal?: AbortSignal;
  /** Called before each attempt with the normalized URL. */
  onAttempt?: (url: string, index: number, total: number) => void;
  /** Delay between attempts (ms). Default 150. */
  backoffMs?: number;
  /** Optional ws path; default '/location'. */
  path?: string;
  /** Override default port; default 8091. */
  port?: string;
}

/** Best single WS URL for location (active profile). */
export function locationWsUrl(opts?: { path?: string; port?: string }): string {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;
  const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('resolved URL', url);
  return url;
}

/** Ordered candidates (active profile → others → same-origin). */
export function locationCandidates(opts?: { path?: string; port?: string }): string[] {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;
  const list = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('candidates', list);
  return list;
}

/** Connect with fallback; returns an OPEN WebSocket or throws a descriptive Error. */
export async function connectLocationWs(
  opts: LocationConnectOpts = {}
): Promise<WebSocket> {
  const {
    timeoutMs = 6000,
    signal,
    onAttempt,
    backoffMs,
    path = DEFAULT_PATH,
    port = DEFAULT_PORT,
  } = opts;

  const candidates = locationCandidates({ path, port });
  if (!candidates.length) {
    throw new Error('No location WebSocket candidates were generated (check env).');
  }

  try {
    const { ws, url } = await connectWithFallback(candidates, timeoutMs, {
      signal,
      onAttempt,
      backoffMs,
    });
    dlog('connected via', url);
    (ws as any)._chosenUrl = url;
    return ws;
  } catch (err: any) {
    const tried = candidates.join(', ');
    const msg = err?.message ? String(err.message) : 'unknown error';
    const e = new Error(
      `Location WS: all candidates failed. Tried: ${tried}. Last error: ${msg}`
    );
    (e as any).cause = err;
    throw e;
  }
}

/** Direct open (no fallback). Returns null if URL missing or constructor throws. */
export function openLocationSocket(opts?: { path?: string; port?: string }): WebSocket | null {
  const url = locationWsUrl(opts);
  if (!url) return null;
  try {
    const ws = new WebSocket(upgradeWsForHttps(url));
    (ws as any)._chosenUrl = url;
    return ws;
  } catch {
    return null;
  }
}
