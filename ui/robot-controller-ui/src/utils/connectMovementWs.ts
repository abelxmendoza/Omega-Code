// src/utils/connectMovementWs.ts

/**
 * Attempts to establish a WebSocket connection to the robot backend.
 * Tries the Tailscale URL first for security, then falls back to LAN if needed.
 *
 * Env variables:
 *   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE
 *   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN
 *
 * Usage:
 *   import { connectMovementWs } from '@/utils/connectMovementWs';
 *   connectMovementWs().then(ws => { ... }).catch(() => { ... });
 */

export async function connectMovementWs(): Promise<WebSocket> {
  const tailscaleUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE;
  const lanUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN;

  if (!tailscaleUrl || !lanUrl) {
    throw new Error(
      "WebSocket URLs are not defined in environment variables. Please set NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE and NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN in your .env.local file."
    );
  }

  return new Promise((resolve, reject) => {
    let settled = false;
    let ws: WebSocket;

    // Try Tailscale URL first
    try {
      ws = new WebSocket(tailscaleUrl);

      ws.onopen = () => {
        settled = true;
        resolve(ws);
      };

      ws.onerror = () => {
        if (!settled) {
          // Try LAN fallback
          try {
            let ws2 = new WebSocket(lanUrl);
            ws2.onopen = () => resolve(ws2);
            ws2.onerror = reject;
          } catch (err) {
            reject(err);
          }
        }
      };
    } catch (err) {
      // If Tailscale immediately throws, try LAN
      try {
        let ws2 = new WebSocket(lanUrl);
        ws2.onopen = () => resolve(ws2);
        ws2.onerror = reject;
      } catch (err2) {
        reject(err2);
      }
    }
  });
}
