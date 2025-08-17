// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectLineTrackerWs.ts
// Summary:
//   Line Tracker WS utilities.
//   - getLineTrackerWsUrl(): profile-resolved single URL
//   - connectLineTrackerWs({ timeoutMs }): fallback connect across candidates
//   - startJsonHeartbeat(ws, ...): JSON ping/pong with latency reporting
//   - parseLineTrackingPayload(raw): normalizes payload to { IR01, IR02, IR03 }

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback } from './wsConnect';

export function getLineTrackerWsUrl(): string {
  const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER');
  if (!url) throw new Error('LineTracker WS URL not set for active profile (.env)');
  return url;
}

export async function connectLineTrackerWs(opts?: { timeoutMs?: number }): Promise<WebSocket> {
  const timeoutMs = opts?.timeoutMs ?? 6000;
  const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER');
  const { ws } = await connectWithFallback(candidates, timeoutMs);
  return ws;
}

/** Start JSON ping/pong heartbeat. Returns stop() cleanup. */
export function startJsonHeartbeat(
  ws: WebSocket,
  opts?: {
    intervalMs?: number; // default 10s
    timeoutMs?: number;  // default 6s; latency -> null if no pong in time
    onLatency?: (ms: number | null) => void;
    onDisconnect?: () => void;
  }
): () => void {
  const intervalMs = opts?.intervalMs ?? 10000;
  const timeoutMs = opts?.timeoutMs ?? 6000;

  let hbTimer: ReturnType<typeof setInterval> | null = null;
  let timeoutHandle: ReturnType<typeof setTimeout> | null = null;
  let pingSentAt: number | null = null;

  const onMessage = (evt: MessageEvent) => {
    try {
      const data = JSON.parse(evt.data as string);
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
  tick(); // fire once immediately
  hbTimer = setInterval(tick, intervalMs);

  return () => {
    ws.removeEventListener('message', onMessage);
    if (hbTimer) clearInterval(hbTimer);
    if (timeoutHandle) clearTimeout(timeoutHandle);
  };
}

/** Normalize server payload to IR01/IR02/IR03. */
export function parseLineTrackingPayload(raw: any):
  | { IR01: number; IR02: number; IR03: number }
  | null {
  try {
    if (raw?.lineTracking) {
      const lt = raw.lineTracking;
      return { IR01: +(lt.left ?? 0), IR02: +(lt.center ?? 0), IR03: +(lt.right ?? 0) };
    }
    if (raw?.sensors) {
      const s = raw.sensors;
      return { IR01: +(s.left ?? 0), IR02: +(s.center ?? 0), IR03: +(s.right ?? 0) };
    }
    if (raw?.IR01 !== undefined && raw?.IR02 !== undefined && raw?.IR03 !== undefined) {
      return { IR01: +raw.IR01, IR02: +raw.IR02, IR03: +raw.IR03 };
    }
  } catch {}
  return null;
}
