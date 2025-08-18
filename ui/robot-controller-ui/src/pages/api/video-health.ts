/*
# File: /Omega-Code/ui/robot-controller-ui/src/pages/api/video-health.ts
# Summary:
#   Same-origin proxy for the upstream /health endpoint used by VideoFeed.
#   - Avoids mixed-content/CORS blocks when the UI is served over HTTPS.
#   - Honors ?profile=lan|tailscale|local and ?video=<override-url>, just like /api/video-proxy.
#   - Robustly derives the health URL from the upstream video URL:
#       .../video_feed[?...]  →  .../health
#       other paths           →  <base>/health
*/

import type { NextApiRequest, NextApiResponse } from 'next';

export const config = {
  api: { bodyParser: false },
};

type Profile = 'lan' | 'tailscale' | 'local';

function pickProfile(req: NextApiRequest): Profile {
  const qp = (typeof req.query.profile === 'string' && req.query.profile.toLowerCase()) || '';
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as Profile[]).includes(env as Profile)
    ? (env as Profile)
    : 'local';
}

function pickUpstreamBase(req: NextApiRequest): string | null {
  // Highest precedence: ?video= override
  const override = typeof req.query.video === 'string' ? req.query.video.trim() : '';
  if (override) return override;

  const prof = pickProfile(req);
  const envByProf: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };

  return (
    envByProf[prof] ||
    envByProf.tailscale ||
    envByProf.lan ||
    envByProf.local ||
    null
  );
}

function toHealthUrl(upstream: string): string {
  try {
    const u = new URL(upstream);
    // If path ends with /video_feed[/], replace with /health
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = ''; // health doesn’t need the stream’s query
    } else {
      // Append /health to the base path
      u.pathname = (u.pathname.replace(/\/+$/,'') || '') + '/health';
    }
    return u.toString();
  } catch {
    // Fallback string replace
    return upstream.replace(/\/video_feed(?:\?.*)?$/i, '') + '/health';
  }
}

async function fetchJsonWithTimeout(url: string, ms: number) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), ms);
  try {
    const r = await fetch(url, { signal: ac.signal });
    const txt = await r.text().catch(() => '');
    let body: any = undefined;
    try { body = txt ? JSON.parse(txt) : undefined; } catch { body = { raw: txt }; }
    return { status: r.status, ok: r.ok, body };
  } finally {
    clearTimeout(t);
  }
}

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  try {
    const upstreamBase = pickUpstreamBase(req);
    if (!upstreamBase) {
      res.status(500).json({ error: 'No video stream URL configured in env.' });
      return;
    }

    const healthUrl = toHealthUrl(upstreamBase);
    const r = await fetchJsonWithTimeout(healthUrl, 2500);

    res.status(r.status).setHeader('Cache-Control', 'no-store');
    res.setHeader('Content-Type', 'application/json; charset=utf-8');

    // Normalize a couple expected shapes for the client:
    // - { ok: true } (200)
    // - { placeholder: true } → treat as "no_camera"
    const payload = {
      upstream: healthUrl,
      ok: r.ok,
      ...((r.body && typeof r.body === 'object') ? r.body : {}),
    };

    res.end(JSON.stringify(payload));
  } catch (err: any) {
    res
      .status(502)
      .setHeader('Content-Type', 'application/json; charset=utf-8')
      .end(JSON.stringify({ error: 'Health proxy failure', detail: String(err?.message || err) }));
  }
}
