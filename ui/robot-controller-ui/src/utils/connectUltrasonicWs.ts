// File: /Omega-Code/ui/robot-controller-ui/src/utils/connectUltrasonicWs.ts
// Summary:
//   Ultrasonic WS helper with graceful fallback across profiles.
//   - getUltrasonicWsUrl(): returns the active-profile URL from env
//   - connectUltrasonicWs({ timeoutMs }): tries [active profile, then others] until one connects
//   - openUltrasonicSocket(): direct opener (no fallback), auto-upgrades ws:// → wss:// when needed
//
// Notes:
//   The ultrasonic stream is typically a push stream without pong. Use your streaming-aware hook
//   (e.g., useWsStatus(url, { treatAnyMessageAsAlive: true })) to track liveness.

'use client';

import { resolveWsUrl, resolveWsCandidates } from './resolveWsUrl';
import { connectWithFallback, upgradeWsForHttps } from './wsConnect';

/** Single resolved WS URL for the active profile (or '' if unset) */
export function getUltrasonicWsUrl(): string {
  return resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');
}

/** Tries profile-preferred URL first, then falls back to others */
export async function connectUltrasonicWs(opts?: { timeoutMs?: number }): Promise<WebSocket> {
  const timeoutMs = opts?.timeoutMs ?? 6000;
  const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');
  const { ws } = await connectWithFallback(candidates, timeoutMs);
  return ws;
}

/** Optional: direct open without fallback */
export function openUltrasonicSocket(): WebSocket | null {
  const url = getUltrasonicWsUrl();
  return url ? new WebSocket(upgradeWsForHttps(url)) : null;
}
