// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectMovementWs.ts
// Summary:
//   Movement WS helper with graceful fallback + robust error handling.
//   - movementWsUrl(): the best single URL for the active profile
//   - movementCandidates(): ordered list of fallback URLs
//   - connectMovementWs(opts): try candidates until one connects; supports AbortSignal
//
// Enable debug logs by setting NEXT_PUBLIC_WS_DEBUG=1

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

const DEFAULT_PORT = '8081';

const DEBUG =
  typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
    (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:movement]', ...a);

export interface MovementConnectOpts {
  /** Per-attempt timeout (ms). Default 6000. */
  timeoutMs?: number;
  /** Abort all remaining attempts (e.g., on component unmount). */
  signal?: AbortSignal;
  /** Called before each attempt with the normalized URL. */
  onAttempt?: (url: string, index: number, total: number) => void;
  /** Delay between attempts (ms). Default 150. */
  backoffMs?: number;
  /** Optional ws path (e.g., '/ws'). */
  path?: string;
  /** Override default port (defaults to 8081). */
  port?: string;
}

/** Best single URL for movement (active profile). */
export function movementWsUrl(opts?: { path?: string; port?: string }): string {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? '';
  const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('resolved URL', url);
  return url;
}

/** Ordered candidates for movement (profile → others → same-origin). */
export function movementCandidates(opts?: { path?: string; port?: string }): string[] {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? '';
  const list = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('candidates', list);
  return list;
}

/** Connect with fallback; returns an OPEN WebSocket or throws a descriptive Error. */
export async function connectMovementWs(
  opts: MovementConnectOpts = {}
): Promise<WebSocket> {
  const {
    timeoutMs = 6000,
    signal,
    onAttempt,
    backoffMs,
    path,
    port = DEFAULT_PORT,
  } = opts;

  const candidates = movementCandidates({ path, port });
  if (!candidates.length) {
    throw new Error('No movement WebSocket candidates were generated (check env).');
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
    // Provide a clear, user-facing message with what we tried.
    const tried = candidates.join(', ');
    const msg = err?.message ? String(err.message) : 'unknown error';
    const e = new Error(
      `Movement WS: all candidates failed. Tried: ${tried}. Last error: ${msg}`
    );
    (e as any).cause = err;
    throw e;
  }
}

/** Optional direct opener (no fallback). Returns null if URL missing or constructor throws. */
export function openMovementSocket(opts?: { path?: string; port?: string }): WebSocket | null {
  const url = movementWsUrl(opts);
  if (!url) return null;
  try {
    const ws = new WebSocket(upgradeWsForHttps(url));
    (ws as any)._chosenUrl = url;
    return ws;
  } catch {
    return null;
  }
}
