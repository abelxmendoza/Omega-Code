// File: src/utils/connectLineTrackerWs.ts
/**
 * Line Tracker WebSocket helper
 *
 * - connectLineTrackerWs(): Promise<WebSocket> — resolves when OPEN
 * - startJsonHeartbeat(ws, { onLatency, onDisconnect, intervalMs, timeoutMs }): () => void
 * - parseLineTrackingPayload(raw): { IR01, IR02, IR03 } | null
 *
 * Env (include full path):
 *  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE=ws://<pi-ip>:8090/line-tracker
 *  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN=ws://<lan-ip>:8090/line-tracker
 *  NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL=ws://localhost:8090/line-tracker (optional)
 *  NEXT_PUBLIC_NETWORK_PROFILE=tailscale|lan|local
 */

import { resolveWsUrl } from '@/utils/resolveWsUrl';

// Prefer profile-based URL, fall back to direct envs if resolver not present/misconfigured.
const LINE_TRACKER_WS =
  resolveWsUrl?.('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER') ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL;

export function getLineTrackerWsUrl(): string {
  if (!LINE_TRACKER_WS) {
    throw new Error('LineTracker WS URL not set in env');
  }
  return LINE_TRACKER_WS;
}

export async function connectLineTrackerWs(opts?: { timeoutMs?: number }): Promise<WebSocket> {
  const url = getLineTrackerWsUrl();
  const timeoutMs = opts?.timeoutMs ?? 6000;

  return new Promise<WebSocket>((resolve, reject) => {
    const ws = new WebSocket(url);
    let settled = false;

    const to = setTimeout(() => {
      if (!settled) {
        settled = true;
        try { ws.close(); } catch {}
        reject(new Error('LineTracker WS connect timeout'));
      }
    }, timeoutMs);

    ws.onopen = () => {
      if (settled) return;
      settled = true;
      clearTimeout(to);
      resolve(ws);
    };

    ws.onerror = (err) => {
      if (settled) return;
      settled = true;
      clearTimeout(to);
      reject(err instanceof Event ? new Error('LineTracker WS error') : (err as any));
    };

    ws.onclose = () => {
      if (settled) return;
      settled = true;
      clearTimeout(to);
      reject(new Error('LineTracker WS closed during connect'));
    };
  });
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
      const data = JSON.parse(evt.data);
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
