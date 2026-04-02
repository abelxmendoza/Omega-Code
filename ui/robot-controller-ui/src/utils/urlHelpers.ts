/*
# File: /src/utils/urlHelpers.ts
# Summary:
#   Shared URL normalization helpers.
#   Extracted from Header.tsx so they can be unit-tested and reused by services.
*/

/**
 * Convert an MJPEG / WebSocket URL to a sibling HTTP `/health` URL.
 *
 * fetch() cannot use ws:// or wss:// — this function always normalizes
 * the scheme to http:// or https:// before returning.
 *
 * Examples:
 *   ws://omegaone:5000/video_feed  → http://omegaone:5000/health
 *   wss://robot:5000/video_feed    → https://robot:5000/health
 *   http://robot:5000/video_feed   → http://robot:5000/health
 *   /api/video-proxy?profile=lan   → /api/video-health?profile=lan
 */
export function toHealthUrl(videoUrl?: string): string {
  if (!videoUrl) return '';
  try {
    const base = typeof window !== 'undefined' ? window.location.origin : 'http://localhost';
    const u = new URL(videoUrl, base);

    // Normalize WS schemes → HTTP so fetch() can use the URL
    if (u.protocol === 'ws:')  u.protocol = 'http:';
    if (u.protocol === 'wss:') u.protocol = 'https:';

    // Same-origin proxy: mirror query to /api/video-health
    if (u.pathname.startsWith('/api/video-proxy')) {
      const qs = new URLSearchParams(u.search);
      qs.delete('b'); // strip cache-buster
      const q = qs.toString();
      return `/api/video-health${q ? `?${q}` : ''}`;
    }

    // Direct upstream: replace /video_feed with /health
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = '';
    } else {
      u.pathname = (u.pathname.replace(/\/+$/, '') || '') + '/health';
    }
    return u.toString();
  } catch {
    // Fallback for malformed URLs: string-replace the scheme and path
    return videoUrl
      .replace(/^ws:\/\//i,  'http://')
      .replace(/^wss:\/\//i, 'https://')
      .replace(/\/video_feed(?:\?.*)?$/i, '')
      .replace(/\/+$/, '') + '/health';
  }
}

/**
 * Normalize a WebSocket URL to its HTTP equivalent so it can be
 * used with fetch() for health probes.
 *
 *   ws://host:port/path  → http://host:port/path
 *   wss://host:port/path → https://host:port/path
 */
export function wsUrlToHttp(url: string): string {
  if (!url) return url;
  return url
    .replace(/^ws:\/\//i,  'http://')
    .replace(/^wss:\/\//i, 'https://');
}
