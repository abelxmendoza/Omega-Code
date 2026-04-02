/*
# File: /src/utils/envProfile.ts
# Summary:
#   Profile-aware environment variable helpers.
#   Extracted from pages/index.tsx so they can be unit-tested and reused.
*/

export type ActiveProfile = 'lan' | 'tailscale' | 'local';

/** Returns the active network profile from NEXT_PUBLIC_NETWORK_PROFILE env var. */
export function getActiveProfile(): ActiveProfile {
  const p = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'].includes(p) ? p : 'local') as ActiveProfile;
}

/**
 * Read an env var by active profile, with LOCAL fallback.
 *
 * Example:
 *   getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT')
 *   → reads NEXT_PUBLIC_BACKEND_WS_URL_LAN (if profile=lan)
 *   → falls back to NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL
 *   → falls back to ''
 */
export function getEnvByProfile(base: string, profile?: string): string {
  const p = (profile ?? getActiveProfile()).toUpperCase();
  return (
    process.env[`${base}_${p}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
}

/** Coerce ws:// → http:// (or wss: → https:) if on an HTTPS page; honor NEXT_PUBLIC_WS_FORCE_INSECURE. */
export function coerceWsScheme(rawUrl: string): string {
  if (!rawUrl) return rawUrl;
  try {
    const u = new URL(rawUrl, typeof window !== 'undefined' ? window.location.href : 'http://localhost');
    const forceInsecure = !!process.env.NEXT_PUBLIC_WS_FORCE_INSECURE;
    if (forceInsecure) {
      if (u.protocol === 'wss:') u.protocol = 'ws:';
      return u.toString();
    }
    if (typeof window !== 'undefined' && window.location.protocol === 'https:' && u.protocol === 'ws:') {
      u.protocol = 'wss:';
    }
    return u.toString();
  } catch {
    return rawUrl;
  }
}
