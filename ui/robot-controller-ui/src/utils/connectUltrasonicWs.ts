// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectUltrasonicWs.ts
// Summary:
//   Ultrasonic WS helper with graceful fallback across profiles.
//   - ultrasonicWsUrl(): best single URL for the active profile
//   - ultrasonicCandidates(): ordered fallback list (profile → others → same-origin)
//   - connectUltrasonicWs(opts): tries candidates until one connects; supports AbortSignal
//   - openUltrasonicSocket(): direct opener (no fallback), auto-upgrades ws:// → wss://
// Notes:
//   Your Go server (main_ultrasonic.go) defaults to PORT 8080 and PATH /ultrasonic.
//   This stream may not emit pong; treat any message as alive in your status hook.
//
// Enable debug logs by setting NEXT_PUBLIC_WS_DEBUG=1

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

const DEFAULT_PORT = '8080';
const DEFAULT_PATH = '/ultrasonic';

const DEBUG =
  typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
    (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:ultrasonic]', ...a);

export interface UltrasonicConnectOpts {
  /** Per-attempt timeout (ms). Default 6000. */
  timeoutMs?: number;
  /** Abort all remaining attempts (e.g., on component unmount). */
  signal?: AbortSignal;
  /** Called before each attempt with the normalized URL. */
  onAttempt?: (url: string, index: number, total: number) => void;
  /** Delay between attempts (ms). Default 150. */
  backoffMs?: number;
  /** Optional ws path (e.g., '/ultrasonic'); default '/ultrasonic'. */
  path?: string;
  /** Override default port (defaults to 8080). */
  port?: string;
}

/** Best single WS URL for ultrasonic (active profile). */
export function ultrasonicWsUrl(opts?: { path?: string; port?: string }): string {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;
  const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('resolved URL', url);
  return url;
}

/** Ordered candidates for ultrasonic (profile → others → same-origin). */
export function ultrasonicCandidates(opts?: { path?: string; port?: string }): string[] {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;
  const list = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('candidates', list);
  return list;
}

/** Connect with fallback; returns an OPEN WebSocket or throws a descriptive Error. */
export async function connectUltrasonicWs(
  opts: UltrasonicConnectOpts = {}
): Promise<WebSocket> {
  const {
    timeoutMs = 6000,
    signal,
    onAttempt,
    backoffMs,
    path = DEFAULT_PATH,
    port = DEFAULT_PORT,
  } = opts;

  const candidates = ultrasonicCandidates({ path, port });
  if (!candidates.length) {
    throw new Error('No ultrasonic WebSocket candidates were generated (check env).');
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
      `Ultrasonic WS: all candidates failed. Tried: ${tried}. Last error: ${msg}`
    );
    (e as any).cause = err;
    throw e;
  }
}

/** Optional direct open (no fallback). Returns null if URL missing or constructor throws. */
export function openUltrasonicSocket(opts?: { path?: string; port?: string }): WebSocket | null {
  const url = ultrasonicWsUrl(opts);
  if (!url) return null;
  try {
    const ws = new WebSocket(upgradeWsForHttps(url));
    (ws as any)._chosenUrl = url;
    return ws;
  } catch {
    return null;
  }
}
