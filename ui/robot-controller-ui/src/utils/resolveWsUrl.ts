//src/utils/resolveWsUrl.ts



/**
 * Resolve a WebSocket URL from env based on NEXT_PUBLIC_NETWORK_PROFILE.
 *
 * Usage:
 *  const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER');
 *  const url2 = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');
 *
 * Env must define per-profile keys like:
 *  <BASE>_LAN, <BASE>_TAILSCALE, <BASE>_LOCAL
 */
export function resolveWsUrl(baseKey: string): string | undefined {
  const profile = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || '').toLowerCase();
  const pick = (suffix: string) => (process.env as any)[`${baseKey}_${suffix}`];

  // Preferred profile first
  let chosen: string | undefined;
  if (profile === 'tailscale') chosen = pick('TAILSCALE') || pick('LAN') || pick('LOCAL');
  else if (profile === 'lan') chosen = pick('LAN') || pick('TAILSCALE') || pick('LOCAL');
  else if (profile === 'local') chosen = pick('LOCAL') || pick('LAN') || pick('TAILSCALE');
  else chosen = pick('TAILSCALE') || pick('LAN') || pick('LOCAL'); // default

  return chosen;
}
