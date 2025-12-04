/**
 * Safe WebSocket Utilities
 * Wrappers that disable robot WebSocket connections when running on Vercel (cloud)
 */

import { ROBOT_ENABLED } from "./env";

export function robotWS(url: string): WebSocket | null {
  if (!ROBOT_ENABLED) {
    console.warn(`[Robot OFFLINE] Blocked WS: ${url}`);
    return null;
  }

  try {
    return new WebSocket(url);
  } catch (err) {
    console.error("[robotWS] failed:", err);
    return null;
  }
}
