/**
 * Safe WebSocket Utilities
 * Wrappers that disable robot WebSocket connections when running on Vercel (cloud)
 */

import { ROBOT_ENABLED } from "./env";

export function robotWS(path: string): WebSocket | null {
  if (!ROBOT_ENABLED) {
    console.warn(`[Robot OFFLINE] Blocked WS connection: ${path}`);
    return null;
  }

  try {
    const ws = new WebSocket(path);
    ws.onerror = (e) => console.error("[RobotWS Error]", e);
    return ws;
  } catch (err) {
    console.error("[RobotWS Fail]", err);
    return null;
  }
}

