// File: /src/utils/connectLightingWs.ts

export function getLightingWsUrl(): string {
  // Prefer Tailscale, fallback to LAN, then localhost
  return (
    process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE ||
    process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN ||
    'ws://localhost:8082/lighting'
  );
}

export function connectLightingWs(): Promise<WebSocket> {
  const tailscaleUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE;
  const lanUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN;

  return new Promise((resolve, reject) => {
    let ws: WebSocket;
    let settled = false;

    // Try Tailscale first
    if (tailscaleUrl) {
      ws = new WebSocket(tailscaleUrl);
      ws.onopen = () => {
        settled = true;
        resolve(ws);
      };
      ws.onerror = () => {
        if (!settled && lanUrl) {
          // Fallback to LAN
          let ws2 = new WebSocket(lanUrl);
          ws2.onopen = () => resolve(ws2);
          ws2.onerror = reject;
        } else {
          reject();
        }
      };
    } else if (lanUrl) {
      ws = new WebSocket(lanUrl);
      ws.onopen = () => resolve(ws);
      ws.onerror = reject;
    } else {
      // Fallback to localhost for dev
      ws = new WebSocket('ws://localhost:8082/lighting');
      ws.onopen = () => resolve(ws);
      ws.onerror = reject;
    }
  });
}
