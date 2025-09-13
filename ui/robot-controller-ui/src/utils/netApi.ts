/*
# File: /Omega-Code/ui/robot-controller-ui/src/utils/netApi.ts
# Summary:
#   Tiny client for the net-ops API (PAN + Wi-Fi + summary).
#   Works with or without NEXT_PUBLIC_NET_API_BASE; when not set, uses same-origin /api/*.
#   Includes small helpers for safe URL building and JSON fetching with useful errors.
#
#   Env:
#     NEXT_PUBLIC_NET_API_BASE = http://<pi>:<port>  (e.g. http://raspberrypi.local:5055)
*/

const RAW_BASE = (process.env.NEXT_PUBLIC_NET_API_BASE || '').trim();

/** Normalize base; allow empty (same-origin). Ensures no trailing slash. */
const BASE = RAW_BASE.replace(/\/+$/, ''); // '' or 'http://host:port'

/** Join base + path (`/api/...`) without double slashes when base is set. */
function joinUrl(path: string): string {
  // path should start with '/'; if not, add it.
  const p = path.startsWith('/') ? path : `/${path}`;
  return BASE ? `${BASE}${p}` : p; // same-origin when BASE is empty
}

/** Fetch → JSON with consistent error shaping. */
async function fetchJson<T>(input: string, init?: RequestInit): Promise<T> {
  let res: Response;
  try {
    res = await fetch(input, init);
  } catch (e: any) {
    // Network / CORS / DNS errors
    throw new Error(`Network error while fetching ${input}: ${e?.message || e}`);
  }

  if (!res.ok) {
    // Try to extract a helpful error body (text first; JSON fallback)
    const text = await res
      .text()
      .catch(() => '')
      .then((t) => t || `HTTP ${res.status}`);
    throw new Error(text);
  }

  // If body is empty, return null as any (some endpoints may be 204)
  const ct = res.headers.get('content-type') || '';
  if (!ct.toLowerCase().includes('application/json')) {
    // Try anyway; if it fails, return null
    try {
      return (await res.json()) as T;
    } catch {
      return null as unknown as T;
    }
  }
  return (await res.json()) as T;
}

export type NetSummary = {
  ssid: string | null;
  ips: string[]; // all IPv4/IPv6 addresses you want to show
  pan: { bnep0: boolean };
  tailscale?: { installed?: boolean; backendState?: string; peerCount?: number };
  env?: { PHONE_MAC?: string | null; PHONE_NAME_FRAGMENT?: string | null };
};

export const netApi = {
  /** GET /api/net/summary */
  async summary(): Promise<NetSummary> {
    const url = joinUrl('/api/net/summary');
    return fetchJson<NetSummary>(url, { cache: 'no-store' });
  },

  /** POST /api/net/pan/connect[?query=...] */
  async connectPan(query?: string): Promise<any> {
    const qs = query ? `?${new URLSearchParams({ query }).toString()}` : '';
    const url = joinUrl(`/api/net/pan/connect${qs}`);
    return fetchJson<any>(url, { method: 'POST' });
  },

  /** POST /api/net/pan/disconnect */
  async disconnectPan(): Promise<any> {
    const url = joinUrl('/api/net/pan/disconnect');
    return fetchJson<any>(url, { method: 'POST' });
  },

  /** POST /api/net/wifi/connect?ssid=&psk=&persistent= */
  async connectWifi(ssid: string, psk: string, persistent = true): Promise<any> {
    const params = new URLSearchParams({
      ssid,
      psk,
      persistent: String(persistent),
    });
    const url = joinUrl(`/api/net/wifi/connect?${params.toString()}`);
    return fetchJson<any>(url, { method: 'POST' });
  },
};
