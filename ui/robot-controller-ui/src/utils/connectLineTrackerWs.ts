// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectLineTrackerWs.ts
// Summary:
//   Line Tracker WS utilities (robust).
//   - lineTrackerWsUrl(): profile-resolved best single URL
//   - lineTrackerCandidates(): ordered fallback list (profile → others → same-origin)
//   - connectLineTrackerWs(opts): fallback connect with timeout/abort/backoff
//   - startJsonHeartbeat(ws, ...): JSON ping/pong with latency reporting
//   - parseLineTrackingPayload(raw): normalizes payload to { IR01, IR02, IR03 }
// Notes:
//   Server defaults: port 8090, path /line-tracker (see your Python server).

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

const DEFAULT_PORT = '8090';
const DEFAULT_PATH = '/line-tracker';

const DEBUG =
  typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
    (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const dlog = (...a: any[]) => DEBUG && console.info('[WS:line-tracker]', ...a);

export interface LineTrackerConnectOpts {
  /** Per-attempt timeout (ms). Default 6000. */
  timeoutMs?: number;
  /** Abort all remaining attempts (e.g., on component unmount). */
  signal?: AbortSignal;
  /** Called before each attempt with the normalized URL. */
  onAttempt?: (url: string, index: number, total: number) => void;
  /** Delay between attempts (ms). Default 150. */
  backoffMs?: number;
  /** Optional ws path; default '/line-tracker'. */
  path?: string;
  /** Override default port; default 8090. */
  port?: string;
}

/** Profile-resolved single URL for the active profile. */
export function lineTrackerWsUrl(opts?: { path?: string; port?: string }): string {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;
  const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('resolved URL', url);
  return url;
}

/** Ordered candidates (active profile → others → same-origin). */
export function lineTrackerCandidates(opts?: { path?: string; port?: string }): string[] {
  const port = opts?.port ?? DEFAULT_PORT;
  const path = opts?.path ?? DEFAULT_PATH;
  const list = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER', {
    defaultPort: port,
    path,
    includeSameOrigin: true,
  });
  dlog('candidates', list);
  return list;
}

/** Connect with fallback; returns an OPEN WebSocket or throws a descriptive Error. */
export async function connectLineTrackerWs(
  opts: LineTrackerConnectOpts = {}
): Promise<WebSocket> {
  const {
    timeoutMs = 6000,
    signal,
    onAttempt,
    backoffMs,
    path = DEFAULT_PATH,
    port = DEFAULT_PORT,
  } = opts;

  const candidates = lineTrackerCandidates({ path, port });
  if (!candidates.length) {
    throw new Error('No line-tracker WebSocket candidates were generated (check env).');
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
      `Line Tracker WS: all candidates failed. Tried: ${tried}. Last error: ${msg}`
    );
    (e as any).cause = err;
    throw e;
  }
}

/** Direct open (no fallback). Returns null if URL missing or constructor throws. */
export function openLineTrackerSocket(opts?: { path?: string; port?: string }): WebSocket | null {
  const url = lineTrackerWsUrl(opts);
  if (!url) return null;
  try {
    const ws = new WebSocket(upgradeWsForHttps(url));
    (ws as any)._chosenUrl = url;
    return ws;
  } catch {
    return null;
  }
}

/** Start JSON ping/pong heartbeat. Returns stop() cleanup. */
export function startJsonHeartbeat(
  ws: WebSocket,
  opts?: {
    /** default 10s */
    intervalMs?: number;
    /** default 6s; latency -> null if no pong in time */
    timeoutMs?: number;
    onLatency?: (ms: number | null) => void;
    onDisconnect?: () => void;
  }
): () => void {
  const intervalMs = opts?.intervalMs ?? 10_000;
  const timeoutMs = opts?.timeoutMs ?? 6_000;

  let hbTimer: ReturnType<typeof setInterval> | null = null;
  let timeoutHandle: ReturnType<typeof setTimeout> | null = null;
  let pingSentAt: number | null = null;

  const onMessage = (evt: MessageEvent) => {
    try {
      const data = typeof evt.data === 'string' ? JSON.parse(evt.data) : evt.data;
      if (data?.type === 'pong') {
        const end = performance.now();
        const start = pingSentAt ?? end;
        opts?.onLatency?.(Math.max(0, Math.round(end - start)));
        pingSentAt = null;
        if (timeoutHandle) {
          clearTimeout(timeoutHandle);
          timeoutHandle = null;
        }
      }
    } catch {
      /* ignore non-JSON */
    }
  };

  const tick = () => {
    if (ws.readyState !== WebSocket.OPEN) {
      opts?.onDisconnect?.();
      return;
    }
    try {
      pingSentAt = performance.now();
      ws.send(JSON.stringify({ type: 'ping', ts: Date.now() }));
      if (timeoutHandle) clearTimeout(timeoutHandle);
      timeoutHandle = setTimeout(() => {
        pingSentAt = null;
        opts?.onLatency?.(null);
      }, timeoutMs);
    } catch {
      opts?.onDisconnect?.();
    }
  };

  ws.addEventListener('message', onMessage);
  // fire once immediately, then repeat
  tick();
  hbTimer = setInterval(tick, intervalMs);

  return () => {
    ws.removeEventListener('message', onMessage);
    if (hbTimer) clearInterval(hbTimer);
    if (timeoutHandle) clearTimeout(timeoutHandle);
  };
}

/** Normalize server payload to IR01/IR02/IR03. Accepts raw string or parsed object. */
export type LineTrackingSample = { IR01: number; IR02: number; IR03: number };

export function parseLineTrackingPayload(raw: unknown): LineTrackingSample | null {
  try {
    const data = typeof raw === 'string' ? JSON.parse(raw) : (raw as any);

    // Server welcome/sample shapes
    if (data?.lineTracking && typeof data.lineTracking === 'object') {
      const lt = data.lineTracking;
      return {
        IR01: +(lt.left ?? lt.Left ?? 0),
        IR02: +(lt.center ?? lt.Center ?? 0),
        IR03: +(lt.right ?? lt.Right ?? 0),
      };
    }

    // Alternate "sensors" shape
    if (data?.sensors && typeof data.sensors === 'object') {
      const s = data.sensors;
      return {
        IR01: +(s.left ?? s.Left ?? 0),
        IR02: +(s.center ?? s.Center ?? 0),
        IR03: +(s.right ?? s.Right ?? 0),
      };
    }

    // Flat keys already normalized
    if (
      data?.IR01 !== undefined &&
      data?.IR02 !== undefined &&
      data?.IR03 !== undefined
    ) {
      return { IR01: +data.IR01, IR02: +data.IR02, IR03: +data.IR03 };
    }
  } catch {
    // ignore parse errors → null
  }
  return null;
}
