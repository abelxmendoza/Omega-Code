/*
# File: /Omega-Code/ui/robot-controller-ui/src/utils/netApi.ts
# Summary: Tiny client for the net-ops API (PAN + Wi-Fi + summary).
# Env:
#   NEXT_PUBLIC_NET_API_BASE=http://<pi>:<port>  (e.g. http://raspberrypi.local:5055)
*/

const BASE = (process.env.NEXT_PUBLIC_NET_API_BASE || '').replace(/\/$/, '');

async function toJson<T>(res: Response): Promise<T> {
  if (!res.ok) {
    const text = await res.text().catch(() => '');
    throw new Error(text || `HTTP ${res.status}`);
  }
  return res.json() as Promise<T>;
}

export type NetSummary = {
  ssid: string | null;
  ips: string[]; // all IPv4/6 you want to show
  pan: { bnep0: boolean };
  tailscale?: { installed?: boolean; backendState?: string; peerCount?: number };
  env?: { PHONE_MAC?: string | null; PHONE_NAME_FRAGMENT?: string | null };
};

export const netApi = {
  summary(): Promise<NetSummary> {
    return toJson(fetch(`${BASE}/api/net/summary`, { cache: 'no-store' }));
  },
  connectPan(query?: string): Promise<any> {
    const url = new URL(`${BASE}/api/net/pan/connect`);
    if (query) url.searchParams.set('query', query);
    return toJson(fetch(url, { method: 'POST' }));
  },
  disconnectPan(): Promise<any> {
    return toJson(fetch(`${BASE}/api/net/pan/disconnect`, { method: 'POST' }));
  },
  connectWifi(ssid: string, psk: string, persistent = true): Promise<any> {
    const url = new URL(`${BASE}/api/net/wifi/connect`);
    url.searchParams.set('ssid', ssid);
    url.searchParams.set('psk', psk);
    url.searchParams.set('persistent', String(persistent));
    return toJson(fetch(url, { method: 'POST' }));
  },
};
