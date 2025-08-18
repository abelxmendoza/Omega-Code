// File: /Omega-Code/ui/robot-controller-ui/src/pages/api/video-proxy.ts
// Summary:
//   Same-origin proxy for your MJPEG stream. Avoids mixed-content blocks when the UI is HTTPS.
//   Priority:
//     1) ?video=<URL> (dev override)
//     2) active profile URL (TAILSCALE / LAN / LOCAL)
//     3) first non-empty of {TAILSCALE, LAN, LOCAL}
//
// Usage from UI: <img src="/api/video-proxy" />
// Optional: /api/video-proxy?profile=tailscale   (overrides active profile)
//           /api/video-proxy?video=http://host:5000/video_feed   (explicit override)
//
// Tip: Set NEXT_PUBLIC_NETWORK_PROFILE in your .env.local to choose the default profile.

import type { NextApiRequest, NextApiResponse } from 'next';
import { Readable } from 'stream';

export const config = {
  api: { bodyParser: false, responseLimit: false },
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

function pickUpstream(req: NextApiRequest): string | null {
  // Highest precedence: ?video= override
  const override = typeof req.query.video === 'string' ? req.query.video.trim() : '';
  if (override) return override;

  const prof = pickProfile(req);

  const envByProf: Record<Profile, string | undefined> = {
    tailscale: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
    lan: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
    local: process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL,
  };

  // Preferred profile first, then fall back to any that exists
  return (
    envByProf[prof] ||
    envByProf.tailscale ||
    envByProf.lan ||
    envByProf.local ||
    null
  );
}

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  try {
    const upstream = pickUpstream(req);
    if (!upstream) {
      res.status(500).json({ error: 'No video stream URL configured in env.' });
      return;
    }

    const r = await fetch(upstream);
    if (!r.ok || !r.body) {
      res.status(502).json({ error: `Upstream error`, status: r.status, url: upstream });
      return;
    }

    // Forward key headers so the browser treats it as MJPEG
    const ct = r.headers.get('content-type') || 'multipart/x-mixed-replace; boundary=frame';
    res.setHeader('Content-Type', ct);
    res.setHeader('Cache-Control', 'no-store, no-transform');
    res.setHeader('Connection', 'keep-alive');

    // Pipe the readable stream to the client; stop when client disconnects
    const nodeStream = Readable.fromWeb(r.body as unknown as ReadableStream);
    req.on('close', () => {
      try { nodeStream.destroy(); } catch {}
    });
    nodeStream.pipe(res);
  } catch (err: any) {
    if (!res.headersSent) {
      res.status(500).json({ error: 'Proxy failure', detail: String(err?.message || err) });
    } else {
      res.end();
    }
  }
}
