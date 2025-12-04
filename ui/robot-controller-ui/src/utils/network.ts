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

export async function robotFetch(
  path: string,
  options?: RequestInit
): Promise<Response | OfflineResponse> {
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
