/*
# File: /Omega-Code/ui/robot-controller-ui/src/pages/api/camera-power.ts
# Summary:
#   Same-origin proxy for the video server's /camera/power endpoint.
#   GET  → {"enabled": bool, "healthy": bool}
#   POST → {"enabled": bool} — turns camera hardware on or off.
#
#   URL resolution mirrors video-proxy.ts: picks the video server base
#   from NEXT_PUBLIC_VIDEO_STREAM_URL_{LAN,TAILSCALE,LOCAL}, stripping
#   the /video_feed suffix so we hit the Flask server root.
*/

import type { NextApiRequest, NextApiResponse } from 'next';

// Next.js body parser is active (no streaming here), so req.body is available.
export const config = { api: { bodyParser: true } };

type Profile = 'lan' | 'tailscale' | 'local';
const TIMEOUT_MS = 4000;

function pickProfile(req: NextApiRequest): Profile {
  const qp = (typeof req.query.profile === 'string' ? req.query.profile.toLowerCase() : '');
  if (qp === 'lan' || qp === 'tailscale' || qp === 'local') return qp as Profile;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as const).includes(env as Profile) ? (env as Profile) : 'local';
}

function isAbsoluteHttpUrl(u: string): boolean {
  try { const x = new URL(u); return x.protocol === 'http:' || x.protocol === 'https:'; }
  catch { return false; }
}

/** Strip /video_feed (and anything after it) to get the Flask server base URL. */
function toServerBase(videoUrl: string): string {
  try {
    const u = new URL(videoUrl);
    u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '').replace(/\/+$/, '') || '/';
    u.search = '';
    return u.toString().replace(/\/+$/, '');
  } catch {
    return videoUrl.replace(/\/video_feed(?:\?.*)?$/i, '').replace(/\/+$/, '');
  }
}

function buildCandidateBases(req: NextApiRequest): string[] {
  const prof = pickProfile(req);
  const by: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan:       process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local:     process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };
  const all: Profile[] = ['tailscale', 'lan', 'local'];
  const order = [prof, ...all.filter(p => p !== prof)];
  const list = order
    .map(p => by[p])
    .filter((v): v is string => !!v && isAbsoluteHttpUrl(v))
    .map(toServerBase);
  return Array.from(new Set(list));
}

async function fetchWithTimeout(url: string, init: RequestInit, ms: number): Promise<Response> {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), ms);
  try {
    return await fetch(url, { ...init, signal: ac.signal });
  } finally {
    clearTimeout(t);
  }
}

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET' && req.method !== 'POST') {
    res.setHeader('Allow', 'GET, POST');
    return res.status(405).json({ ok: false, error: 'method_not_allowed' });
  }

  // Mock mode: camera always "enabled" in dev
  if (process.env.MOCK_BACKEND === '1') {
    if (req.method === 'GET') {
      return res.status(200).json({ ok: true, enabled: true, healthy: true, mock: true });
    }
    const want = (req.body as any)?.enabled;
    return res.status(200).json({ ok: true, enabled: !!want, healthy: !!want, mock: true });
  }

  const bases = buildCandidateBases(req);
  if (bases.length === 0) {
    return res.status(500).json({
      ok: false,
      error: 'missing_upstream',
      hint: 'Set NEXT_PUBLIC_VIDEO_STREAM_URL_* in .env.local',
    });
  }

  const body = req.method === 'POST' ? JSON.stringify(req.body) : undefined;

  for (const base of bases) {
    const upstreamUrl = `${base}/camera/power`;
    try {
      const upRes = await fetchWithTimeout(
        upstreamUrl,
        {
          method: req.method,
          headers: { 'Content-Type': 'application/json' },
          body,
          cache: 'no-store',
        },
        TIMEOUT_MS,
      );
      const json = await upRes.json().catch(() => ({ ok: false }));
      return res
        .status(upRes.status)
        .setHeader('Cache-Control', 'no-store, must-revalidate')
        .json(json);
    } catch {
      // try next candidate
    }
  }

  return res.status(502).json({
    ok: false,
    error: 'proxy_failure',
    detail: 'All upstream candidates failed',
  });
}
