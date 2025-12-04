/**
 * Safe Network Utilities
 * Wrappers that disable robot API calls when running on Vercel (cloud)
 */

import { ROBOT_ENABLED, ROBOT_BASE_URL } from "./env";

export async function robotFetch(path: string, opts: RequestInit = {}) {
  if (!ROBOT_ENABLED) {
    console.warn(`[Robot OFFLINE] Blocked fetch: ${path}`);
    return { ok: false, offline: true } as Response;
  }

  try {
    const url = `${ROBOT_BASE_URL}${path}`;
    const r = await fetch(url, opts);
    return r;
  } catch (err) {
    console.error(`[RobotFetch Error]`, err);
    return { ok: false, error: err } as Response;
  }
}

