// File: /Omega-Code/ui/robot-controller-ui/src/pages/api/video-proxy.ts
import type { NextApiRequest, NextApiResponse } from 'next';
import dns from 'node:dns';
import http from 'node:http';
import https from 'node:https';
import { URL } from 'node:url';

export const config = { api: { bodyParser: false, responseLimit: false } };

type Profile = 'lan' | 'tailscale' | 'local';

/* ----------------------------- helpers ------------------------------ */

function pickProfile(req: NextApiRequest): Profile {
  const qp = (typeof req.query.profile === 'string' && req.query.profile.toLowerCase()) || '';
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp as Profile;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as const).includes(env as Profile) ? (env as Profile) : 'local';
}

function buildStreamCandidates(req: NextApiRequest): string[] {
  const override = typeof req.query.video === 'string' ? req.query.video.trim() : '';
  if (override) return [override];

  const prof = pickProfile(req);
  const by: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan:       process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local:     process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };

  const all: Profile[] = ['tailscale', 'lan', 'local'];
  const order = [prof, ...all.filter(p => p !== prof)];
  const list  = order.map(p => by[p]).filter(Boolean) as string[];

  return Array.from(new Set(list));
}

function errorShape(err: any) {
  const c = err?.cause || err;
  return { message: String(err?.message || err), code: c?.code, errno: c?.errno, syscall: c?.syscall };
}

function streamOnce(
  upstream: string,
  opts: { ipv4First: boolean; headerTimeoutMs: number },
  req: NextApiRequest,
  res: NextApiResponse
): Promise<void> {
  return new Promise((resolve, reject) => {
    let responded = false;
    let headerTimer: NodeJS.Timeout | null = null;

    const url = new URL(upstream);
    const isHttps = url.protocol === 'https:';
    const mod = isHttps ? https : http;

    const lookup: typeof dns.lookup | undefined = opts.ipv4First
      ? ((hostname, options, cb) => {
          const o = typeof options === 'object' ? options : {};
          dns.lookup(hostname, { ...o, family: 4, verbatim: false }, cb as any);
        })
      : undefined;

    const agent = new (isHttps ? https.Agent : http.Agent)({
      keepAlive: true,
      // @ts-expect-error: http.request passes lookup through agent options
      lookup,
    });

    const request = mod.request(
      {
        protocol: url.protocol,
        hostname: url.hostname,
        port: url.port || (isHttps ? 443 : 80),
        path: url.pathname + url.search,
        method: 'GET',
        agent,
        headers: {
          Accept: 'multipart/x-mixed-replace, image/jpeg;q=0.9,*/*;q=0.8',
          Connection: 'keep-alive',
        },
        timeout: opts.headerTimeoutMs,
      },
      (upRes) => {
        const status = upRes.statusCode || 0;
        if (status < 200 || status >= 400) {
          reject(new Error(`Upstream status ${status}`));
          return;
        }

        responded = true;
        if (headerTimer) clearTimeout(headerTimer);

        const ct = upRes.headers['content-type'] || 'multipart/x-mixed-replace; boundary=frame';
        res.setHeader('Content-Type', Array.isArray(ct) ? ct[0] : ct);
        res.setHeader('Cache-Control', 'no-store, no-transform');
        res.setHeader('Connection', 'keep-alive');

        upRes.on('error', () => { try { res.end(); } catch {} });
        req.on('close', () => { try { upRes.destroy(); } catch {} });

        upRes.pipe(res);
        resolve();
      }
    );

    headerTimer = setTimeout(() => {
      if (!responded) {
        try { request.destroy(new Error('Header timeout')); } catch {}
      }
    }, opts.headerTimeoutMs + 50);

    request.on('error', (e) => {
      if (headerTimer) clearTimeout(headerTimer);
      reject(e);
    });

    req.on('close', () => { try { request.destroy(new Error('Client closed')); } catch {} });

    request.end();
  });
}

/* ---------------------------------- API ----------------------------------- */

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method && req.method !== 'GET') {
    res.setHeader('Allow', 'GET');
    res.status(405).json({ error: 'Method Not Allowed' });
    return;
  }

  const streams = buildStreamCandidates(req);
  if (streams.length === 0) {
    res.status(500).json({ error: 'No video stream URL configured. Set NEXT_PUBLIC_VIDEO_STREAM_URL_* in .env.local' });
    return;
  }

  const tried: Array<{ upstream: string; attempt: 'ipv4first' | 'default'; error: any }> = [];
  const HEADER_TIMEOUT_MS = 3500;

  for (const upstream of streams) {
    try {
      await streamOnce(upstream, { ipv4First: true, headerTimeoutMs: HEADER_TIMEOUT_MS }, req, res);
      return; // streaming
    } catch (e1) {
      tried.push({ upstream, attempt: 'ipv4first', error: errorShape(e1) });
    }
    try {
      await streamOnce(upstream, { ipv4First: false, headerTimeoutMs: HEADER_TIMEOUT_MS }, req, res);
      return; // streaming
    } catch (e2) {
      tried.push({ upstream, attempt: 'default', error: errorShape(e2) });
    }
  }

  if (!res.headersSent) {
    res
      .status(502)
      .setHeader('Content-Type', 'application/json; charset=utf-8')
      .setHeader('Cache-Control', 'no-store, must-revalidate')
      .end(JSON.stringify({
        error: 'Proxy failure',
        detail: 'All upstream candidates failed',
        tried,
        ts: Date.now(),
      }));
    return;
  }

  try { res.end(); } catch {}
}
