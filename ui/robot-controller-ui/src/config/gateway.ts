/**
 * Gateway Configuration
 * Centralized gateway URL building with profile-aware resolution
 */

import type { NetProfile } from '@/utils/resolveWsUrl';

/**
 * Get active profile (works on both server and client)
 */
function getActiveProfile(): NetProfile {
  if (typeof window === 'undefined') {
    // Server-side: always use environment variable or default
    const raw = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
    return (['lan', 'tailscale', 'local'] as NetProfile[]).includes(raw as NetProfile)
      ? (raw as NetProfile)
      : 'local';
  }
  
  // Client-side: check URL override first, then environment
  try {
    const q = new URL(window.location.href).searchParams.get('profile');
    if (q && ['lan', 'tailscale', 'local'].includes(q)) return q as NetProfile;
  } catch {}
  
  const raw = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'] as NetProfile[]).includes(raw as NetProfile)
    ? (raw as NetProfile)
    : 'local';
}

export interface GatewayConfig {
  host: string;
  port: string;
  scheme: string;
  baseUrl: string;
}

/**
 * Get gateway configuration based on active profile
 */
function getGatewayConfig(): GatewayConfig {
  const profile = getActiveProfile();
  
  // Profile-specific host overrides
  let host: string | undefined;
  let port: string | undefined;
  let scheme: string | undefined;
  
  switch (profile) {
    case 'tailscale':
      host = process.env.NEXT_PUBLIC_ROBOT_HOST_TAILSCALE || process.env.NEXT_PUBLIC_GATEWAY_HOST_TAILSCALE;
      port = process.env.NEXT_PUBLIC_GATEWAY_PORT_TAILSCALE || process.env.NEXT_PUBLIC_GATEWAY_PORT;
      scheme = process.env.NEXT_PUBLIC_GATEWAY_SCHEME_TAILSCALE || process.env.NEXT_PUBLIC_GATEWAY_SCHEME;
      break;
    case 'lan':
      host = process.env.NEXT_PUBLIC_ROBOT_HOST_LAN || process.env.NEXT_PUBLIC_GATEWAY_HOST_LAN;
      port = process.env.NEXT_PUBLIC_GATEWAY_PORT_LAN || process.env.NEXT_PUBLIC_GATEWAY_PORT;
      scheme = process.env.NEXT_PUBLIC_GATEWAY_SCHEME_LAN || process.env.NEXT_PUBLIC_GATEWAY_SCHEME;
      break;
    case 'local':
      host = process.env.NEXT_PUBLIC_ROBOT_HOST_LOCAL || process.env.NEXT_PUBLIC_GATEWAY_HOST_LOCAL;
      port = process.env.NEXT_PUBLIC_GATEWAY_PORT_LOCAL || process.env.NEXT_PUBLIC_GATEWAY_PORT;
      scheme = process.env.NEXT_PUBLIC_GATEWAY_SCHEME_LOCAL || process.env.NEXT_PUBLIC_GATEWAY_SCHEME;
      break;
  }
  
  // Fallback to generic gateway host
  host = host || process.env.NEXT_PUBLIC_GATEWAY_HOST || 'localhost';
  port = port || process.env.NEXT_PUBLIC_GATEWAY_PORT || '7070';
  scheme = (scheme || process.env.NEXT_PUBLIC_GATEWAY_SCHEME || 'http').toLowerCase();
  
  // If host includes scheme, extract it
  if (/^https?:\/\//i.test(host)) {
    const url = new URL(host);
    scheme = url.protocol.replace(':', '');
    host = url.hostname;
    if (url.port) port = url.port;
  }
  
  const baseUrl = `${scheme}://${host}${port ? `:${port}` : ''}`;
  
  return { host, port, scheme, baseUrl };
}

/**
 * Build a gateway URL for a given path
 */
export function buildGatewayUrl(path: string, options?: { profile?: NetProfile }): string {
  const config = options?.profile 
    ? getGatewayConfigForProfile(options.profile)
    : getGatewayConfig();
  
  const cleanPath = path.startsWith('/') ? path : `/${path}`;
  return `${config.baseUrl}${cleanPath}`;
}

/**
 * Get gateway config for a specific profile (useful for server-side)
 */
function getGatewayConfigForProfile(profile: NetProfile): GatewayConfig {
  let host: string | undefined;
  let port: string | undefined;
  let scheme: string | undefined;
  
  switch (profile) {
    case 'tailscale':
      host = process.env.NEXT_PUBLIC_ROBOT_HOST_TAILSCALE || process.env.NEXT_PUBLIC_GATEWAY_HOST_TAILSCALE;
      port = process.env.NEXT_PUBLIC_GATEWAY_PORT_TAILSCALE || process.env.NEXT_PUBLIC_GATEWAY_PORT;
      scheme = process.env.NEXT_PUBLIC_GATEWAY_SCHEME_TAILSCALE || process.env.NEXT_PUBLIC_GATEWAY_SCHEME;
      break;
    case 'lan':
      host = process.env.NEXT_PUBLIC_ROBOT_HOST_LAN || process.env.NEXT_PUBLIC_GATEWAY_HOST_LAN;
      port = process.env.NEXT_PUBLIC_GATEWAY_PORT_LAN || process.env.NEXT_PUBLIC_GATEWAY_PORT;
      scheme = process.env.NEXT_PUBLIC_GATEWAY_SCHEME_LAN || process.env.NEXT_PUBLIC_GATEWAY_SCHEME;
      break;
    case 'local':
      host = process.env.NEXT_PUBLIC_ROBOT_HOST_LOCAL || process.env.NEXT_PUBLIC_GATEWAY_HOST_LOCAL;
      port = process.env.NEXT_PUBLIC_GATEWAY_PORT_LOCAL || process.env.NEXT_PUBLIC_GATEWAY_PORT;
      scheme = process.env.NEXT_PUBLIC_GATEWAY_SCHEME_LOCAL || process.env.NEXT_PUBLIC_GATEWAY_SCHEME;
      break;
  }
  
  host = host || process.env.NEXT_PUBLIC_GATEWAY_HOST || 'localhost';
  port = port || process.env.NEXT_PUBLIC_GATEWAY_PORT || '7070';
  scheme = (scheme || process.env.NEXT_PUBLIC_GATEWAY_SCHEME || 'http').toLowerCase();
  
  if (/^https?:\/\//i.test(host)) {
    const url = new URL(host);
    scheme = url.protocol.replace(':', '');
    host = url.hostname;
    if (url.port) port = url.port;
  }
  
  const baseUrl = `${scheme}://${host}${port ? `:${port}` : ''}`;
  
  return { host, port, scheme, baseUrl };
}

/**
 * Get the current gateway configuration
 */
export function getGatewayConfigForCurrentProfile(): GatewayConfig {
  return getGatewayConfig();
}

/**
 * Export default config for convenience
 */
export const gatewayConfig = getGatewayConfig();

