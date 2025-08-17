// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectMovementWs.ts
// Summary:
//   Movement WS helper with graceful fallback.
//   - connectMovementWs({ timeoutMs }): tries [active profile, then others] until one connects
//   - movementWsUrl(): single resolved URL for the active profile

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

export const movementWsUrl = (): string =>
  resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');

export async function connectMovementWs(opts?: { timeoutMs?: number }): Promise<WebSocket> {
  const timeoutMs = opts?.timeoutMs ?? 6000;
  const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');
  const { ws } = await connectWithFallback(candidates, timeoutMs);
  return ws;
}

/** Optional direct opener (without fallback) */
export function openMovementSocket(): WebSocket | null {
  const url = movementWsUrl();
  return url ? new WebSocket(upgradeWsForHttps(url)) : null;
}
