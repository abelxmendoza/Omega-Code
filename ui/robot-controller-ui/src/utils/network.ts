// --------------------------------------------------
// Robot Offline Mode – Final Stable Blueprint
// --------------------------------------------------

import { ROBOT_ENABLED, ROBOT_BASE_URL } from "./env";

//
// 1. Unified OfflineResponse Type
//

export interface OfflineResponse {
  ok: false;
  offline: true;
  status: number;
  statusText: string;
  json(): Promise<any>;
  text(): Promise<string>;
}

//
// 2. Factory for fake Response-like offline responses
//

export function createOfflineResponse(path: string): OfflineResponse {
  return {
    ok: false,
    offline: true,
    status: 503, // Service Unavailable
    statusText: "Service Unavailable",
    json: async () => ({
      offline: true,
      message: `Robot backend unavailable: ${path}`,
    }),
    text: async () => `Robot backend unavailable: ${path}`,
  };
}

//
// 3. Wrapper for fetch() — returns OfflineResponse on Vercel
//

/**
 * A regular fetch Response extended to indicate it is NOT offline.
 * Allows callers to use `if (response.offline)` without a type cast,
 * since both union members expose the `offline` discriminant.
 */
export type RobotResponse = OfflineResponse | (Response & { offline?: never });

export async function robotFetch(
  path: string,
  options?: RequestInit
): Promise<RobotResponse> {
  if (!ROBOT_ENABLED) {
    console.warn(`[Robot OFFLINE] Blocked fetch: ${path}`);
    return createOfflineResponse(path);
  }

  try {
    const url = `${ROBOT_BASE_URL}${path}`;
    return await fetch(url, options);
  } catch (err) {
    console.error(`[RobotFetch ERROR] ${path}`, err);
    return createOfflineResponse(path);
  }
}

//
// 4. Safe WebSocket factory — respects ROBOT_ENABLED offline guard
//

export function robotWS(url: string): WebSocket | null {
  if (!ROBOT_ENABLED) {
    console.warn(`[Robot OFFLINE] Blocked WS: ${url}`);
    return null;
  }
  try {
    return new WebSocket(url);
  } catch (err) {
    console.error('[robotWS] failed:', err);
    return null;
  }
}
