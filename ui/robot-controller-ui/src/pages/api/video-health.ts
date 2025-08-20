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
#   Note:
#   - Uses native fetch (Node 18+) with an AbortController timeout.
#   - No 'undici' import needed.
*/

import type { NextApiRequest, NextApiResponse } from 'next';

export const config = { api: { bodyParser: false } };

type Profile = 'lan' | 'tailscale' | 'local';

/* ----------------------------- helpers ------------------------------ */

function pickProfile(req: NextApiRequest): Profile {
  const qp = (typeof req.query.profile === 'string' && req.query.profile.toLowerCase()) || '';
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp as Profile;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as const).includes(env as Profile) ? (env as Profile) : 'local';
}

function toHealthUrl(upstream: string): string {
  try {
    const u = new URL(upstream);
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = '';
    } else {
      u.pathname = (u.pathname.replace(/\/+$/,'') || '') + '/health';
    }
    return u.toString();
  } catch {
    return upstream.replace(/\/video_feed(?:\?.*)?$/i, '') + '/health';
  }
}

/** Build an ordered list of upstream *video* URLs (later converted to /health). */
function buildUpstreamCandidates(req: NextApiRequest): string[] {
  // 1) explicit override
  const override = typeof req.query.video === 'string' ? req.query.video.trim() : '';
  if (override) return [override];

  // 2) active profile, then the others
  const prof = pickProfile(req);
  const by: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan:       process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local:     process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };

  const all: Profile[] = ['tailscale', 'lan', 'local'];
  const order = [prof, ...all.filter(p => p !== prof)];
  const list  = order.map(p => by[p]).filter(Boolean) as string[];

  // de-dup
  return Array.from(new Set(list));
}

async function fetchJsonWithTimeout(url: string, ms: number) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), ms);
  try {
    const r = await fetch(url, { signal: ac.signal, cache: 'no-store' });
    const text = await r.text().catch(() => '');
    let body: any = undefined;
    try { body = text ? JSON.parse(text) : undefined; } catch { body = { raw: text }; }
    return { status: r.status, ok: r.ok, body };
  } finally {
    clearTimeout(t);
  }
}

/* ---------------------------------- API ----------------------------------- */

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method && req.method !== 'GET') {
    res.setHeader('Allow', 'GET');
    res.status(405).json({ error: 'Method Not Allowed' });
    return;
  }

  const upstreams = buildUpstreamCandidates(req);
  if (upstreams.length === 0) {
    res
      .status(500)
      .json({ error: 'No video stream URL configured in env.' });
    return;
  }

  const tried: Array<{ health: string; err?: string; status?: number }> = [];

  for (const base of upstreams) {
    const healthUrl = toHealthUrl(base);
    try {
      const r = await fetchJsonWithTimeout(healthUrl, 2500);
      // Normalize for the client:
      // 200 → ok: true
      // 503 + { placeholder: true } → “no_camera” signal on client
      res.status(r.status).setHeader('Cache-Control', 'no-store');
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      const payload = {
        upstream: healthUrl,
        ok: r.ok,
        ...((r.body && typeof r.body === 'object') ? r.body : {}),
      };
      res.end(JSON.stringify(payload));
      return;
    } catch (e: any) {
      tried.push({ health: healthUrl, err: String(e?.message || e) });
    }
  }

  // all failed
  res
    .status(502)
    .setHeader('Content-Type', 'application/json; charset=utf-8')
    .end(JSON.stringify({
      error: 'Health proxy failure',
      detail: 'All upstream candidates failed',
      tried,
      ok: false,
      ts: Date.now(),
    }));
}
