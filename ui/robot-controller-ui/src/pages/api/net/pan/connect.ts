/*
# File: /src/pages/api/net/pan/connect.ts
# Summary:
#   Connect-to-iPhone-PAN proxy. Mockable via MOCK_BACKEND=1.
#   - POST { macOrName?: string }
#   - Forwards to http(s)://<GATEWAY_HOST[:PORT]>/api/net/pan/connect
#   - Honors ?timeout=ms (clamped), never leaks internals
*/

import type { NextApiRequest, NextApiResponse } from 'next';
import { buildGatewayUrl } from '@/config/gateway';

export const config = { api: { bodyParser: true } };

const MOCK = process.env.MOCK_BACKEND === '1';

function clamp(n: number, lo: number, hi: number) {
  return Math.min(hi, Math.max(lo, n));
}

function cleanErr(e: unknown) {
  const msg = (e as any)?.message ?? String(e);
  return { error: 'pan_connect_failed', detail: msg };
}

async function fetchWithTimeout(url: string, init: RequestInit, ms: number) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), ms);
  try {
    return await fetch(url, { ...init, signal: ac.signal, cache: 'no-store' });
  } finally { clearTimeout(t); }
}

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    res.setHeader('Allow', 'POST');
    return res.status(405).json({ error: 'Method Not Allowed' });
  }

  const { macOrName } = (req.body ?? {}) as { macOrName?: string };

  // Optional timeout override (?timeout=ms), sane bounds 500..10000
  const rawTimeout = Array.isArray(req.query.timeout) ? req.query.timeout[0] : req.query.timeout;
  const timeoutMs  = clamp(Number(rawTimeout) || 3500, 500, 10_000);

  // Mock mode: respond quickly without touching gateway
  if (MOCK) {
    await new Promise(r => setTimeout(r, 250));
    res.setHeader('Cache-Control', 'no-store');
    return res.status(200).json({ ok: true, message: `Mock PAN connect${macOrName ? ` (${macOrName})` : ''}` });
  }

  const url = buildGatewayUrl('/api/net/pan/connect');

  try {
    const upstream = await fetchWithTimeout(url, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ macOrName }),
    }, timeoutMs);

    const text = await upstream.text().catch(() => '');
    let body: any = undefined;
    try { body = text ? JSON.parse(text) : undefined; } catch { body = { raw: text }; }

    res.status(upstream.status).setHeader('Cache-Control', 'no-store');
    return res.json(body ?? { ok: upstream.ok });
  } catch (e) {
    return res.status(502).json(cleanErr(e));
  }
}
