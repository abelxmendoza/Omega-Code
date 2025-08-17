// File: /Omega-Code/ui/robot-controller-ui/src/utils/wsConnect.ts
// Summary:
//   Shared helpers for WebSocket connection logic.
//   - upgradeWsForHttps(): auto-upgrade ws:// → wss:// when the page is served over HTTPS
//   - openSocket(url, timeoutMs): Promise that resolves when the socket is OPEN (or rejects on error/timeout)
//   - connectWithFallback(candidates, timeoutMs): tries candidate URLs in order until one succeeds

'use client';

export function upgradeWsForHttps(url: string): string {
  if (!url) return url;
  if (typeof window === 'undefined') return url;          // SSR no-op
  if (window.location.protocol !== 'https:') return url;  // only upgrade when app is served via HTTPS
  try {
    const u = new URL(url);
    if (u.protocol === 'ws:') u.protocol = 'wss:';
    return u.toString();
  } catch {
    return url;
  }
}

export function openSocket(url: string, timeoutMs = 6000): Promise<WebSocket> {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(upgradeWsForHttps(url));
    let settled = false;

    const onOpen = () => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      cleanup();
      resolve(ws);
    };

    const onErr = (ev: Event) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      cleanup();
      reject(new Error(`WS error for ${url}`));
    };

    const onClose = () => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      cleanup();
      reject(new Error(`WS closed during connect: ${url}`));
    };

    const cleanup = () => {
      ws.removeEventListener('open', onOpen);
      ws.removeEventListener('error', onErr);
      ws.removeEventListener('close', onClose);
    };

    ws.addEventListener('open', onOpen);
    ws.addEventListener('error', onErr);
    ws.addEventListener('close', onClose);

    const timer = setTimeout(() => {
      if (settled) return;
      settled = true;
      cleanup();
      try { ws.close(); } catch {}
      reject(new Error(`WS connect timeout: ${url}`));
    }, timeoutMs);
  });
}

/**
 * Tries each candidate URL (in order) until one connects.
 * Skips blanks/duplicates automatically.
 */
export async function connectWithFallback(
  candidates: string[],
  timeoutMs = 6000
): Promise<{ ws: WebSocket; url: string }> {
  const seen = new Set<string>();
  const list = candidates.filter(Boolean).filter(u => {
    const key = upgradeWsForHttps(u);
    if (seen.has(key)) return false;
    seen.add(key);
    return true;
  });

  let lastErr: unknown = null;
  for (const url of list) {
    try {
      const ws = await openSocket(url, timeoutMs);
      return { ws, url };
    } catch (err) {
      lastErr = err;
      // try next candidate
    }
  }
  throw (lastErr instanceof Error ? lastErr : new Error('All WS candidates failed'));
}

