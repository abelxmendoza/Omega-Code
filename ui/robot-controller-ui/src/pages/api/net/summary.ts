/*
# File: /src/pages/api/net/summary.ts
# Summary:
#   Server-side proxy the UI hits for the “Live Link” pill. It forwards to your
#   gateway’s `/api/net/summary` with a short timeout and returns the upstream
#   status + JSON (or an error payload on failure).
#
# Why:
#   - Avoid mixed content/CORS from the browser by calling server→gateway.
#   - Keep the UI decoupled from gateway host/port; use env to switch profiles.
#
# Env (examples in .env.local):
#   NEXT_PUBLIC_GATEWAY_HOST=omega1-1.hartley-ghost.ts.net
#   NEXT_PUBLIC_GATEWAY_PORT=7070
#   # Optional: http|https (defaults to http)
#   # NEXT_PUBLIC_GATEWAY_SCHEME=http
#
# Behavior:
#   - GET/HEAD only (405 otherwise).
#   - Timeout: default 3500ms, override with ?timeout=ms (clamped 500–10000).
#   - Cache-Control: no-store.
#   - On success: pass through status + JSON body (or {} if not JSON).
#   - On failure: 502 with { error, detail, upstream } for graceful UI degrade.
*/

import type { NextApiRequest, NextApiResponse } from 'next';
import { buildGatewayUrl } from '@/config/gateway';

/** Small fetch timeout helper (AbortController pattern). */
async function fetchWithTimeout(url: string, ms: number) {
  const ctrl = new AbortController();
  const t = setTimeout(() => ctrl.abort(), ms);
  try {
    return await fetch(url, { cache: 'no-store', signal: ctrl.signal });
  } finally {
    clearTimeout(t);
  }
}

function clamp(n: number, lo: number, hi: number) {
  return Math.min(hi, Math.max(lo, n));
}

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  // Method guard
  if (req.method !== 'GET' && req.method !== 'HEAD') {
    res.setHeader('Allow', 'GET, HEAD');
    return res.status(405).json({ error: 'method_not_allowed' });
  }

  // ─── Pi-free dev: return canned JSON when MOCK_BACKEND=1 ─────────────────────
  // This lets the Header's Live Link pill work even with no gateway/Pi running.
  // Toggle in `.env.local`: MOCK_BACKEND=1
  if (process.env.MOCK_BACKEND === '1') {
    res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate');
    return res.status(200).json({
      linkType: 'wifi',
      online: true,
      ssid: process.env.MOCK_SSID || 'MockNet',
      ifname: 'wlan0',
      ipv4: '192.168.1.123',
      gateway: '192.168.1.1',
      rssi: -55,
      _mock: true,
    });
  }

  // Always no-store so the pill is fresh
  res.setHeader('Cache-Control', 'no-store, no-cache, must-revalidate, proxy-revalidate');

  const upstreamUrl = buildGatewayUrl('/api/net/summary');
  if (!upstreamUrl || upstreamUrl.includes('localhost') && process.env.NEXT_PUBLIC_GATEWAY_HOST === '') {
    return res.status(502).json({
      error: 'gateway_unconfigured',
      detail: 'NEXT_PUBLIC_GATEWAY_HOST or NEXT_PUBLIC_ROBOT_HOST_* is missing or invalid.',
      upstream: '(unset)',
    });
  }

  // Allow ?timeout=ms override within sane bounds
  const rawTimeout = Array.isArray(req.query.timeout) ? req.query.timeout[0] : req.query.timeout;
  const timeoutMs = clamp(Number(rawTimeout) || 3500, 500, 10_000);

  try {
    const upstream = await fetchWithTimeout(upstreamUrl, timeoutMs);

    if (req.method === 'HEAD') {
      return res.status(upstream.status).end();
    }

    let data: unknown = {};
    try {
      data = await upstream.json();
    } catch {
      try {
        const txt = await upstream.text();
        data = txt ? { note: 'non_json_upstream', raw: txt.slice(0, 512) } : {};
      } catch {
        // swallow
      }
    }

    return res.status(upstream.status).json(data);
  } catch (err: any) {
    const isAbort = err?.name === 'AbortError';
    return res.status(502).json({
      error: isAbort ? 'gateway_timeout' : 'gateway_unreachable',
      detail: String(err?.message || err),
      upstream: upstreamUrl,
    });
  }
}
