// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectLightingWs.ts
// Summary:
//   Lighting WS helper with graceful fallback.
//   - getLightingWsUrl(): returns the best URL for the active profile
//   - connectLightingWs({ timeoutMs }): tries [profile, then others] until one connects
//   - Uses HTTPS-aware upgrade (ws:// → wss://)

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

export function getLightingWsUrl(): string {
  return resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING');
}

export async function connectLightingWs(opts?: { timeoutMs?: number }): Promise<WebSocket> {
  const timeoutMs = opts?.timeoutMs ?? 6000;
  const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING');
  const { ws } = await connectWithFallback(candidates, timeoutMs);
  return ws;
}

/** Optional: direct opener if you just want the preferred single URL */
export function openLightingSocket(): WebSocket | null {
  const url = getLightingWsUrl();
  return url ? new WebSocket(upgradeWsForHttps(url)) : null;
}

