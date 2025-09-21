// File: /Omega-Code/ui/robot-controller-ui/src/utils/wsConnect.ts
// Summary:
//   Shared helpers for WebSocket connection logic.
//   - upgradeWsForHttps(): auto-upgrade ws:// → wss:// when page is HTTPS
//   - openSocket(url, timeoutMs, signal): resolves on OPEN (rejects on error/timeout/abort)
//   - connectWithFallback(candidates, timeoutMs, opts): tries candidates in order until one succeeds
//
// Set NEXT_PUBLIC_WS_DEBUG=1 to enable console logs.

'use client';

const DEBUG = typeof window !== 'undefined' &&
  (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
   (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1');

const log = {
  info: (...a: any[]) => DEBUG && console.info('[WS]', ...a),
  warn: (...a: any[]) => DEBUG && console.warn('[WS]', ...a),
  error: (...a: any[]) => DEBUG && console.error('[WS]', ...a),
};

export function upgradeWsForHttps(url: string): string {
  if (!url) return url;
  if (typeof window === 'undefined') return url; // SSR no-op
  // Fast path (avoid URL ctor churn) for common case.
  if (window.location.protocol === 'https:' && url.startsWith('ws://')) {
    return 'wss://' + url.slice('ws://'.length);
  }
  // Fallback: handle ws:/wss:/http:/https: via URL
  try {
    const u = new URL(url);
    if (window.location.protocol === 'https:' && u.protocol === 'ws:') u.protocol = 'wss:';
    return u.toString();
  } catch {
    // If it's not a full URL (e.g., "omega1:8081"), leave it—caller should provide a full URL.
    return url;
  }
}

export interface OpenOpts {
  timeoutMs?: number;
  signal?: AbortSignal;
}

/** Opens a WebSocket and resolves once it's OPEN; rejects on error/close/timeout/abort. */
export function openSocket(url: string, opts: OpenOpts = {}): Promise<WebSocket> {
  const timeoutMs = opts.timeoutMs ?? 6000;
  const normalized = upgradeWsForHttps(url);

  return new Promise((resolve, reject) => {
    let settled = false;
    let timer: any;

    const ws = new WebSocket(normalized);

    const cleanup = () => {
      ws.removeEventListener('open', onOpen);
      ws.removeEventListener('error', onErr);
      ws.removeEventListener('close', onClose);
      opts.signal?.removeEventListener('abort', onAbort);
      if (timer) clearTimeout(timer);
    };

    const settle = (ok: boolean, value?: any) => {
      if (settled) return;
      settled = true;
      cleanup();
      if (ok) resolve(value); else reject(value);
    };

    const onOpen = () => {
      log.info('open', normalized);
      settle(true, ws);
    };

    const onErr = () => {
      log.warn('error', normalized);
      try { ws.close(); } catch {}
      settle(false, new Error(`WS error: ${normalized}`));
    };

    const onClose = () => {
      log.warn('closed', normalized);
      settle(false, new Error(`WS closed during connect: ${normalized}`));
    };

    const onAbort = () => {
      log.warn('aborted', normalized);
      try { ws.close(); } catch {}
      settle(false, new DOMException('Aborted', 'AbortError'));
    };

    ws.addEventListener('open', onOpen);
    ws.addEventListener('error', onErr);
    ws.addEventListener('close', onClose);
    if (opts.signal) {
      if (opts.signal.aborted) return onAbort();
      opts.signal.addEventListener('abort', onAbort);
    }

    timer = setTimeout(() => {
      log.warn('timeout', normalized);
      try { ws.close(); } catch {}
      settle(false, new Error(`WS connect timeout: ${normalized}`));
    }, timeoutMs);
  });
}

export interface FallbackOpts {
  timeoutMs?: number;
  /** called before each attempt */
  onAttempt?: (url: string, index: number, total: number) => void;
  /** Abort all remaining attempts (e.g., component unmount) */
  signal?: AbortSignal;
  /** delay between attempts (ms) */
  backoffMs?: number;
}

/**
 * Tries each candidate URL (order matters) until one connects.
 * Skips blanks/duplicates automatically. Returns { ws, url } where `url` is the normalized URL used.
 */
export async function connectWithFallback(
  candidates: string[],
  timeoutMs = 6000,
  opts: FallbackOpts = {}
): Promise<{ ws: WebSocket; url: string; index: number }> {
  // Normalize + de-dupe while preserving order
  const seen = new Set<string>();
  const list = candidates
    .filter(Boolean)
    .map(upgradeWsForHttps)
    .filter((u) => (seen.has(u) ? false : (seen.add(u), true)));

  log.info('candidates', list);

  const total = list.length;
  let lastErr: unknown = new Error('No candidates provided');

  for (let i = 0; i < total; i++) {
    const url = list[i];
    if (opts.signal?.aborted) throw new DOMException('Aborted', 'AbortError');
    opts.onAttempt?.(url, i, total);

    try {
      const controller = new AbortController();
      const abortSub = () => controller.abort();
      opts.signal?.addEventListener('abort', abortSub, { once: true });

      const ws = await openSocket(url, { timeoutMs, signal: controller.signal });
      opts.signal?.removeEventListener('abort', abortSub);
      (ws as any)._chosenUrl = url;
      return { ws, url, index: i };
    } catch (err) {
      lastErr = err;
      log.warn(`attempt ${i + 1}/${total} failed`, url, err);
      // small jitter/backoff before next attempt
      const delay = opts.backoffMs ?? 150;
      if (delay > 0) await new Promise((r) => setTimeout(r, delay));
      continue;
    }
  }
  log.error('all attempts failed');
  throw (lastErr instanceof Error ? lastErr : new Error('All WS candidates failed'));
}
