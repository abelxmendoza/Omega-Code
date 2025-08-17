// File: /Omega-Code/ui/robot-controller-ui/src/utils/resolveWsUrl.ts
// Summary:
//   Profile-aware URL resolver for WS/HTTP endpoints.
//   - Reads NEXT_PUBLIC_NETWORK_PROFILE (lan|tailscale|local) with optional ?profile= override.
//   - Resolves env bases (e.g., NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING) to a concrete URL.
//   - Exposes candidate lists for graceful fallback connection attempts.

'use client';

export type NetProfile = 'lan' | 'tailscale' | 'local';

function getUrlProfileOverride(): NetProfile | null {
  if (typeof window === 'undefined') return null;
  const p = new URLSearchParams(window.location.search).get('profile')?.toLowerCase();
  return p === 'lan' || p === 'tailscale' || p === 'local' ? p : null;
}

export function getActiveProfile(): NetProfile {
  const fromUrl = getUrlProfileOverride();
  if (fromUrl) return fromUrl;
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as NetProfile[]).includes(env as NetProfile) ? (env as NetProfile) : 'local';
}

function pick(baseKey: string, prof: NetProfile): string {
  const key = `${baseKey}_${prof.toUpperCase()}`.replace(/-/g, '_');
  // @ts-ignore – Next.js inlines NEXT_PUBLIC_* at build-time
  return (process.env[key] as string | undefined) || '';
}

/**
 * Returns the single URL for the active profile, or '' if unset.
 * Example: resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING')
 */
export function resolveWsUrl(baseKey: WsKey): string {
  return pick(baseKey, getActiveProfile());
}

/**
 * Returns candidate URLs in preferred order:
 *   [active profile first, then the other profiles]
 * Useful for fallback strategies.
 */
export function resolveWsCandidates(baseKey: WsKey): string[] {
  const prof = getActiveProfile();
  const order: NetProfile[] =
    prof === 'tailscale' ? ['tailscale', 'lan', 'local'] :
    prof === 'lan'       ? ['lan', 'tailscale', 'local'] :
                           ['local', 'lan', 'tailscale'];

  return order.map(p => pick(baseKey, p));
}

/** All supported env bases (compile-time safe so Next inlines them). */
export type WsKey =
  | 'NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LOCATION'
  | 'NEXT_PUBLIC_VIDEO_STREAM_URL';
