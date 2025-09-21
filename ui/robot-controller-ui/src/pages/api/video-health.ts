/*
# File: /Omega-Code/ui/robot-controller-ui/src/pages/api/video-health.ts
# Summary:
#   Same-origin proxy for the upstream /health endpoint used by CameraFrame.
#   - Avoids mixed-content/CORS when UI is HTTPS.
#   - Honors ?profile=lan|tailscale|local and ?video=<override-url>.
#   - Robustly derives the health URL from the upstream video URL:
#       .../video_feed[?...]  →  .../health
#       other paths           →  <base>/health
#
# Extras:
#   - Optional ?timeout=2500 (ms), clamped to 500..7000
#   - Adds latencyMs + profile in the response for your UI badge
*/

import type { NextApiRequest, NextApiResponse } from 'next';

export const config = { api: { bodyParser: false } };

type Profile = 'lan' | 'tailscale' | 'local';
const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG;

/* ----------------------------- helpers ------------------------------ */

function debug(...args: any[]) { if (DEBUG) console.log('[video-health]', ...args); }

function pickProfile(req: NextApiRequest): Profile {
  const qp = (typeof req.query.profile === 'string' && req.query.profile.toLowerCase()) || '';
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp as Profile;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as const).includes(env as Profile) ? (env as Profile) : 'local';
}

function clamp(n: number, lo: number, hi: number) { return Math.min(hi, Math.max(lo, n)); }

function getTimeoutMs(req: NextApiRequest): number {
  const raw = typeof req.query.timeout === 'string' ? Number(req.query.timeout) : NaN;
  return clamp(Number.isFinite(raw) ? raw : 2500, 500, 7000);
}

function isAbsoluteHttpUrl(u: string): boolean {
  try { const x = new URL(u); return x.protocol === 'http:' || x.protocol === 'https:'; }
  catch { return false; }
}

function toHealthUrl(upstream: string): string {
  try {
    const u = new URL(upstream);
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = '';
    } else {
      u.pathname = (u.pathname.replace(/\/+$/, '') || '') + '/health';
    }
    return u.toString();
  } catch {
    // Fallback (best-effort) for malformed input
    return upstream.replace(/\/video_feed(?:\?.*)?$/i, '') + '/health';
  }
}

/** Build an ordered list of upstream *video* URLs (later converted to /health). */
function buildUpstreamCandidates(req: NextApiRequest): string[] {
  // 1) explicit override
  const override = typeof req.query.video === 'string' ? req.query.video.trim() : '';
  if (override && isAbsoluteHttpUrl(override)) return [override];

  // 2) active profile, then the others
  const prof = pickProfile(req);
  const by: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan:       process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local:     process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };

  const all: Profile[] = ['tailscale', 'lan', 'local'];
  const order = [prof, ...all.filter(p => p !== prof)];
  const list  = order.map(p => by[p]).filter((v): v is string => !!v && isAbsoluteHttpUrl(v));

  // de-dup
  return Array.from(new Set(list));
}

async function fetchJsonWithTimeout(url: string, ms: number) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), ms);
  const started = Date.now();
  try {
    const r = await fetch(url, { signal: ac.signal, cache: 'no-store' });
    const text = await r.text().catch(() => '');
    let body: any = undefined;
    try { body = text ? JSON.parse(text) : undefined; } catch { body = { raw: text }; }
    return { status: r.status, ok: r.ok, body, latencyMs: Date.now() - started };
  } finally {
    clearTimeout(t);
  }
}

/* ---------------------------------- API ----------------------------------- */

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method && req.method !== 'GET') {
    res.setHeader('Allow', 'GET');
    return res.status(405).json({ error: 'Method Not Allowed' });
  }

  const profile = pickProfile(req);
  const upstreams = buildUpstreamCandidates(req);
  const timeoutMs = getTimeoutMs(req);

  if (upstreams.length === 0) {
    return res
      .status(500)
      .json({ ok: false, error: 'no_upstream_config', hint: 'Set NEXT_PUBLIC_VIDEO_STREAM_URL_* in .env.local', profile });
  }

  const tried: Array<{ health: string; status?: number; err?: string; latencyMs?: number }> = [];

  for (const base of upstreams) {
    const healthUrl = toHealthUrl(base);
    try {
      const r = await fetchJsonWithTimeout(healthUrl, timeoutMs);
      debug('health', r.status, r.ok, r.latencyMs, healthUrl);

      res
        .status(r.status)
        .setHeader('Cache-Control', 'no-store, must-revalidate')
        .setHeader('Content-Type', 'application/json; charset=utf-8')
        .end(JSON.stringify({
          ok: r.ok,
          upstream: healthUrl,
          profile,
          latencyMs: r.latencyMs,
          ...(r.body && typeof r.body === 'object' ? r.body : {}),
        }));
      return;
    } catch (e: any) {
      const errShape = String(e?.message || e);
      tried.push({ health: healthUrl, err: errShape });
      debug('health fail', errShape, healthUrl);
    }
  }

  // all failed
  res
    .status(502)
    .setHeader('Content-Type', 'application/json; charset=utf-8')
    .end(JSON.stringify({
      ok: false,
      error: 'health_proxy_failure',
      detail: 'All upstream candidates failed',
      tried,
      profile,
      ts: Date.now(),
    }));
}
