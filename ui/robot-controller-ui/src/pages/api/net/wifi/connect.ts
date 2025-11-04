/*
# File: /src/pages/api/net/wifi/connect.ts
# Summary:
#   Wi-Fi connect proxy. Mockable via MOCK_BACKEND=1.
#   - POST { ssid: string, psk: string }
#   - Forwards to http(s)://<GATEWAY_HOST[:PORT]>/api/net/wifi/connect
#   - Honors ?timeout=ms (clamped), never echoes PSK back
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
  return { error: 'wifi_connect_failed', detail: msg };
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

  const { ssid, psk } = (req.body ?? {}) as { ssid?: string; psk?: string };
  if (!ssid || typeof ssid !== 'string' || typeof psk !== 'string') {
    return res.status(400).json({ error: 'invalid_payload', detail: 'ssid and psk required' });
  }

  // Optional timeout override (?timeout=ms), sane bounds 500..10000
  const rawTimeout = Array.isArray(req.query.timeout) ? req.query.timeout[0] : req.query.timeout;
  const timeoutMs  = clamp(Number(rawTimeout) || 4000, 500, 10_000);

  if (MOCK) {
    await new Promise(r => setTimeout(r, 300));
    res.setHeader('Cache-Control', 'no-store');
    return res.status(200).json({ ok: true, message: `Mock Wi-Fi connect to "${ssid}" requested` });
  }

  const url = buildGatewayUrl('/api/net/wifi/connect');

  try {
    const upstream = await fetchWithTimeout(url, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      // Never log or return the PSK; just forward.
      body: JSON.stringify({ ssid, psk }),
    }, timeoutMs);

    const text = await upstream.text().catch(() => '');
    let body: any = undefined;
    try { body = text ? JSON.parse(text) : undefined; } catch { body = { raw: text }; }

    // Redact any PSK the gateway might (incorrectly) echo
    if (body && typeof body === 'object' && 'psk' in body) {
      body.psk = '*****';
    }

    res.status(upstream.status).setHeader('Cache-Control', 'no-store');
    return res.json(body ?? { ok: upstream.ok, message: `Wi-Fi connect requested for "${ssid}"` });
  } catch (e) {
    return res.status(502).json(cleanErr(e));
  }
}
