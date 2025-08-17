// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectLocationWs.ts
// Summary:
//   Location WS helper with graceful fallback across profiles.
//   - getLocationWsUrl(): returns the active-profile URL from env
//   - connectLocationWs({ timeoutMs }): tries [active profile, then others] until one connects
//   - openLocationSocket(): direct opener (no fallback), auto-upgrades ws:// → wss:// when needed
//
// Notes:
//   If your location feed supports ping/pong, wire it to your heartbeat hook to get latency.

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

/** Single resolved WS URL for the active profile (or '' if unset) */
export function getLocationWsUrl(): string {
  return resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION');
}

/** Tries profile-preferred URL first, then falls back to others */
export async function connectLocationWs(opts?: { timeoutMs?: number }): Promise<WebSocket> {
  const timeoutMs = opts?.timeoutMs ?? 6000;
  const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION');
  const { ws } = await connectWithFallback(candidates, timeoutMs);
  return ws;
}

/** Optional: direct open without fallback */
export function openLocationSocket(): WebSocket | null {
  const url = getLocationWsUrl();
  return url ? new WebSocket(upgradeWsForHttps(url)) : null;
}
