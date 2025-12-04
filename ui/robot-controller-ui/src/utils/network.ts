/**
 * Safe Network Utilities
 * Wrappers that disable robot API calls when running on Vercel (cloud)
 */

import { ROBOT_ENABLED, ROBOT_BASE_URL } from "./env";

// -----------------------------
// Types
// -----------------------------

export interface OfflineResponse {
  ok: false;
  offline: true;
  status: 0;
  json: () => Promise<any>;
  text: () => Promise<string>;
}

function createOfflineResponse(path: string): OfflineResponse {
  console.warn(`[Robot OFFLINE] Blocked fetch: ${path}`);
  return {
    ok: false,
    offline: true,
    status: 0,
    json: async () => ({ offline: true }),
    text: async () => "offline",
  };
}

// -----------------------------
// Safe fetch wrapper
// -----------------------------

export async function robotFetch(path: string, options?: RequestInit) {
  // BLOCK all robot calls on Vercel
  if (!ROBOT_ENABLED) return createOfflineResponse(path);

  try {
    const url = `${ROBOT_BASE_URL}${path}`;
    const res = await fetch(url, options);
    return res;
  } catch (err) {
    console.error("[robotFetch] error:", err);
    return createOfflineResponse(path);
  }
}
