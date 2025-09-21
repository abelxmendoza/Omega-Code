/*
# File: /Omega-Code/ui/robot-controller-ui/src/pages/api/video-proxy.ts
# Summary:
#   Same-origin MJPEG proxy (avoids mixed-content/CORS). Chooses upstream by profile,
#   tries IPv4-first DNS, then default lookup, and can follow a small number of redirects.
#
#   Query:
#     • ?profile=lan|tailscale|local  → overrides env profile
#     • ?video=<absolute-http(s)-url> → forces a specific upstream
#     • ?mock=1                       → stream a built-in placeholder MJPEG (Pi-free dev)
#     • ?fps=5                        → mock frame rate (default 5)
#
#   Env (URLs per profile; keep absolute http/https):
#     NEXT_PUBLIC_VIDEO_STREAM_URL_LAN
#     NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE
#     NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
#
#   Notes:
#     • We use Node http/https with keep-alive agents for low overhead.
#     • `timeout` + headerTimer guard early stalls (headers never arrive).
#     • Adds X-Accel-Buffering: no (helps behind nginx to not buffer stream).
*/

import type { NextApiRequest, NextApiResponse } from 'next';
import dns from 'node:dns';
import http from 'node:http';
import https from 'node:https';
import { URL } from 'node:url';
import fs from 'node:fs';
import path from 'node:path';

export const config = { api: { bodyParser: false, responseLimit: false } };

type Profile = 'lan' | 'tailscale' | 'local';

const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG; // reuse your existing debug flag
const HEADER_TIMEOUT_MS = 3500;
const MAX_REDIRECTS = 3;
const BOUNDARY = 'frame';

/* ----------------------------- helpers ------------------------------ */

function logDebug(...args: any[]) {
  if (DEBUG) console.log('[video-proxy]', ...args); // eslint-disable-line no-console
}

function pickProfile(req: NextApiRequest): Profile {
  const qp = (typeof req.query.profile === 'string' && req.query.profile.toLowerCase()) || '';
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp as Profile;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as const).includes(env as Profile) ? (env as Profile) : 'local';
}

function isAbsoluteHttpUrl(u: string): boolean {
  try {
    const url = new URL(u);
    return url.protocol === 'http:' || url.protocol === 'https:';
  } catch {
    return false;
  }
}

function buildStreamCandidates(req: NextApiRequest): string[] {
  const override = typeof req.query.video === 'string' ? req.query.video.trim() : '';
  const list: string[] = [];

  if (override && isAbsoluteHttpUrl(override)) list.push(override);

  const prof = pickProfile(req);
  const by: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan:       process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local:     process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };

  const all: Profile[] = ['tailscale', 'lan', 'local'];
  const order = [prof, ...all.filter(p => p !== prof)];
  for (const p of order) {
    const u = by[p];
    if (u && isAbsoluteHttpUrl(u)) list.push(u);
  }

  // de-dup while preserving order
  return Array.from(new Set(list));
}

function errorShape(err: any) {
  const c = err?.cause || err;
  return {
    message: String(err?.message || err),
    code: c?.code,
    errno: c?.errno,
    syscall: c?.syscall,
    name: err?.name,
  };
}

/* ------------------------------ MOCK STREAM --------------------------- */
/** Try public/mock.jpg; fall back to a tiny inline 1x1 JPEG. */
let CACHED_MOCK_BUF: Buffer | null = null;
function getMockFrame(): Buffer {
  if (CACHED_MOCK_BUF) return CACHED_MOCK_BUF;
  try {
    const p = path.join(process.cwd(), 'public', 'mock.jpg');
    CACHED_MOCK_BUF = fs.readFileSync(p);
    logDebug('mock: using public/mock.jpg');
    return CACHED_MOCK_BUF;
  } catch {
    // 1x1 black JPEG (baseline) inline
    const ONE_BY_ONE_JPEG_BASE64 =
      '/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAkGBxAQEBAQEA8PDw8QDw8QDw8PDw8QFREWFhURFRUYHSggGBolGxUVITEhJSkrLi4uFx8zODMtNygtLisBCgoKDg0OGxAQGy0lICUtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLf/AABEIAHAAcAMBIgACEQEDEQH/xAAaAAADAQEBAQAAAAAAAAAAAAAEBQYDAgcB/8QAMhAAAQMDAgQFBAMAAAAAAAAAAQIDBAAFEQYSITFBURMiYXGBkRQjMlLB0fAzYoL/xAAaAQADAQEBAQAAAAAAAAAAAAABAgMABAUH/8QAHxEBAAIDAQEAAwAAAAAAAAAAAAECERIhAzFBIlGh/9oADAMBAAIRAxEAPwD2XAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAADQQW4mjXG2kq2m0nU7l3nO0pXk7yq5JY+0r0e9Vb9CwY/7r2b3h7f1w5m2p5u+Q3c3k2c1f3y0yX6yN8k3l9W1S0i0e2b3d2gS4nKq5fM0M2o3m8i5Jb8f9r5Z4i3n3m9f8A9K0j3o5Tnn9m0+1e7S1z7HjW8Z1V8T2Gz7rBG0lAAAAAAAAAAAAAAAAAABKqkqWJt0c1e8a3z2v0mW5Zk0a5y0fqpv9xTq0b3V6ZdT8f7t8bXbY3f6m3bH2l8m9u2z3d32P2N9o7K3m9Yb9ZpU3kY0s0Wc2k9m9bqO9aZ1JmV3fDWrq0bY8+8cT2W5q9sQqV8Y2b7f0rW+K2dG7cG0nAAAAAAAAAAAAAAAAAABQv0q1q9bL2ulXkzTzq8x3Zf1c1pUV5U1n1q2s7cY9xV5m5c8m9H1k9m6xq6T9LQv1b2rL3WqV7lG8o5l1U0m2Z1s1l8r5tO1e9b2fL9H1gkAAAAAAAAAAAAAAAAAAAB//9k=';
    CACHED_MOCK_BUF = Buffer.from(ONE_BY_ONE_JPEG_BASE64, 'base64');
    logDebug('mock: using inline 1x1 JPEG');
    return CACHED_MOCK_BUF;
  }
}

function isMock(req: NextApiRequest) {
  const q = String(req.query.mock || '').trim();
  if (q === '1' || q.toLowerCase() === 'true') return true;
  return process.env.MOCK_BACKEND === '1';
}

function parseFps(req: NextApiRequest) {
  const raw = Number(req.query.fps);
  return Number.isFinite(raw) && raw > 0 && raw <= 30 ? Math.floor(raw) : 5;
}

function streamMock(req: NextApiRequest, res: NextApiResponse) {
  const fps = parseFps(req);
  const periodMs = Math.max(50, Math.floor(1000 / fps));
  const frame = getMockFrame();

  res.status(200);
  res.setHeader('Content-Type', `multipart/x-mixed-replace; boundary=${BOUNDARY}`);
  res.setHeader('Cache-Control', 'no-store, no-transform');
  res.setHeader('Connection', 'keep-alive');
  res.setHeader('X-Accel-Buffering', 'no');
  res.setHeader('X-Content-Type-Options', 'nosniff');

  let timer: NodeJS.Timeout | null = null;
  let closed = false;

  const writeFrame = () => {
    if (closed) return;
    try {
      res.write(`--${BOUNDARY}\r\n`);
      res.write('Content-Type: image/jpeg\r\n');
      res.write(`Content-Length: ${frame.length}\r\n\r\n`);
      res.write(frame);
      res.write('\r\n');
    } catch {
      // client likely closed mid-write
      cleanup();
    }
  };

  const cleanup = () => {
    if (closed) return;
    closed = true;
    if (timer) clearInterval(timer);
    try { res.end(`--${BOUNDARY}--\r\n`); } catch {}
  };

  // start ticking
  writeFrame();
  timer = setInterval(writeFrame, periodMs);

  req.once('close', cleanup);
  res.once('close', cleanup);
}

/* ------------------------------ REAL STREAM --------------------------- */

// Minimal compatibility type for dns.lookup callback signature
type LookupFnCompat = (
  hostname: string,
  options: number | dns.LookupOneOptions | dns.LookupAllOptions | dns.LookupOptions,
  cb: (...args: any[]) => void
) => void;

type StreamOpts = {
  ipv4First: boolean;
  headerTimeoutMs: number;
  redirectDepth?: number;
};

function streamOnce(
  upstream: string,
  opts: StreamOpts,
  req: NextApiRequest,
  res: NextApiResponse
): Promise<void> {
  return new Promise((resolve, reject) => {
    const redirectDepth = opts.redirectDepth ?? 0;
    let responded = false;
    let headerTimer: NodeJS.Timeout | null = null;

    let url: URL;
    try {
      url = new URL(upstream);
    } catch {
      return reject(new Error(`Invalid upstream URL: ${upstream}`));
    }
    const isHttps = url.protocol === 'https:';
    const mod = isHttps ? https : http;

    // Force IPv4 lookup when requested
    const lookup: LookupFnCompat | undefined = opts.ipv4First
      ? ((hostname, options, cb) => {
          const o = typeof options === 'object' ? options : {};
          dns.lookup(hostname, { ...(o as object), family: 4, verbatim: false }, cb as any);
        })
      : undefined;

    // Keep-alive agent for efficiency
    const agent = new (isHttps ? https.Agent : http.Agent)({ keepAlive: true });

    const request = mod.request(
      {
        protocol: url.protocol,
        hostname: url.hostname,
        port: url.port || (isHttps ? 443 : 80),
        path: url.pathname + url.search,
        method: 'GET',
        agent,
        headers: {
          // Some mjpeg servers are picky; advertise multipart + jpeg
          Accept: 'multipart/x-mixed-replace, image/jpeg;q=0.9,*/*;q=0.8',
          Connection: 'keep-alive',
        },
        timeout: opts.headerTimeoutMs, // socket timeout (not the whole stream)
        lookup: lookup as any,
      },
      (upRes) => {
        const status = upRes.statusCode || 0;
        const loc = upRes.headers.location;

        // Handle small chain of redirects (common with auth/cameras)
        if (status >= 300 && status < 400 && loc && redirectDepth < MAX_REDIRECTS) {
          try {
            const nextUrl = new URL(loc, url).toString();
            logDebug('redirect', { from: url.toString(), to: nextUrl, status });
            // Drain and follow
            upRes.resume();
            if (headerTimer) { clearTimeout(headerTimer); headerTimer = null; }
            request.destroy(); // close current
            return streamOnce(nextUrl, { ...opts, redirectDepth: redirectDepth + 1 }, req, res)
              .then(resolve)
              .catch(reject);
          } catch (e) {
            if (headerTimer) { clearTimeout(headerTimer); headerTimer = null; }
            return reject(new Error(`Bad redirect location: ${String(loc)}`));
          }
        }

        if (status < 200 || status >= 400) {
          if (headerTimer) { clearTimeout(headerTimer); headerTimer = null; }
          const msg = `Upstream status ${status}`;
          // Consume to free socket
          upRes.resume();
          return reject(new Error(msg));
        }

        responded = true;
        if (headerTimer) {
          clearTimeout(headerTimer);
          headerTimer = null;
        }

        // Propagate stream-friendly headers
        const ct = upRes.headers['content-type'] || `multipart/x-mixed-replace; boundary=${BOUNDARY}`;
        res.setHeader('Content-Type', Array.isArray(ct) ? ct[0] : ct);
        res.setHeader('Cache-Control', 'no-store, no-transform');
        res.setHeader('Connection', 'keep-alive');
        res.setHeader('X-Accel-Buffering', 'no'); // prevent buffering on nginx
        res.setHeader('X-Content-Type-Options', 'nosniff');

        // Mirror client close → tear down upstream
        const onClientClose = () => {
          try { upRes.destroy(); } catch {}
        };
        req.once('close', onClientClose);

        upRes.on('error', (e) => {
          logDebug('upstream error', errorShape(e));
          try { res.end(); } catch {}
        });

        // Pipe through (backpressure handled by Node streams)
        upRes.pipe(res);

        // When response ends normally, cleanup listener
        res.once('close', () => {
          req.off('close', onClientClose);
        });

        resolve();
      }
    );

    // Guard for headers never arriving
    headerTimer = setTimeout(() => {
      if (!responded) {
        logDebug('header timeout');
        try { request.destroy(new Error('Header timeout')); } catch {}
      }
    }, opts.headerTimeoutMs + 50);

    // Extra safety: if socket idles before headers, abort
    request.on('timeout', () => {
      if (!responded) {
        logDebug('socket timeout (pre-headers)');
        try { request.destroy(new Error('Socket timeout')); } catch {}
      }
    });

    request.on('error', (e) => {
      if (headerTimer) {
        clearTimeout(headerTimer);
        headerTimer = null;
      }
      reject(e);
    });

    // If the client disconnects before headers, abort the attempt
    req.on('close', () => {
      if (!responded) {
        logDebug('client closed before headers');
        try { request.destroy(new Error('Client closed')); } catch {}
      }
    });

    request.end();
  });
}

/* ---------------------------------- API ----------------------------------- */

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method && req.method !== 'GET' && req.method !== 'HEAD') {
    res.setHeader('Allow', 'GET, HEAD');
    res.status(405).json({ error: 'method_not_allowed' });
    return;
  }

  // HEAD → light “is it configured?” check (mock always OK)
  if (req.method === 'HEAD') {
    if (isMock(req)) { res.status(200).end(); return; }
    const candidates = buildStreamCandidates(req);
    res.status(candidates.length ? 200 : 400).end();
    return;
  }

  // Mock stream for Pi-free dev
  if (isMock(req)) {
    streamMock(req, res);
    return;
  }

  const streams = buildStreamCandidates(req);
  if (streams.length === 0) {
    res.status(500).json({
      error: 'missing_upstream',
      hint: 'Set NEXT_PUBLIC_VIDEO_STREAM_URL_* in .env.local or pass ?video=<absolute url>',
    });
    return;
  }

  const tried: Array<{ upstream: string; attempt: 'ipv4first' | 'default'; error: any }> = [];

  for (const upstream of streams) {
    logDebug('candidate', upstream);

    // Try IPv4-first resolution
    try {
      await streamOnce(upstream, { ipv4First: true, headerTimeoutMs: HEADER_TIMEOUT_MS }, req, res);
      return; // streaming …
    } catch (e1) {
      tried.push({ upstream, attempt: 'ipv4first', error: errorShape(e1) });
      logDebug('ipv4first failed', upstream, errorShape(e1));
    }

    // Then default lookup
    try {
      await streamOnce(upstream, { ipv4First: false, headerTimeoutMs: HEADER_TIMEOUT_MS }, req, res);
      return; // streaming …
    } catch (e2) {
      tried.push({ upstream, attempt: 'default', error: errorShape(e2) });
      logDebug('default lookup failed', upstream, errorShape(e2));
    }
  }

  if (!res.headersSent) {
    res
      .status(502)
      .setHeader('Content-Type', 'application/json; charset=utf-8')
      .setHeader('Cache-Control', 'no-store, must-revalidate')
      .end(JSON.stringify({
        error: 'proxy_failure',
        detail: 'All upstream candidates failed',
        tried,
        ts: Date.now(),
      }));
    return;
  }

  // If headers were already sent, just ensure we close.
  try { res.end(); } catch {}
}
