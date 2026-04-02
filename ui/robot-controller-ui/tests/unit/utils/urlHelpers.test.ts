/*
# File: tests/unit/utils/urlHelpers.test.ts
# Tests: toHealthUrl, wsUrlToHttp
#
# toHealthUrl — converts MJPEG/WS URLs to sibling /health endpoints.
#   Critical invariant: fetch() cannot use ws:// — this function MUST always
#   return an http:// or https:// URL.
#
# wsUrlToHttp — simple scheme coercion: ws:→http:, wss:→https:
*/

import { toHealthUrl, wsUrlToHttp } from '@/utils/urlHelpers';

// ─────────────────────────────────────────────────────────────
// wsUrlToHttp
// ─────────────────────────────────────────────────────────────

describe('wsUrlToHttp', () => {
  it('converts ws:// to http://', () => {
    expect(wsUrlToHttp('ws://robot:5000/ws')).toBe('http://robot:5000/ws');
  });

  it('converts wss:// to https://', () => {
    expect(wsUrlToHttp('wss://robot:5000/ws')).toBe('https://robot:5000/ws');
  });

  it('leaves http:// unchanged', () => {
    expect(wsUrlToHttp('http://robot:5000/health')).toBe('http://robot:5000/health');
  });

  it('leaves https:// unchanged', () => {
    expect(wsUrlToHttp('https://robot:5000/health')).toBe('https://robot:5000/health');
  });

  it('returns empty string unchanged', () => {
    expect(wsUrlToHttp('')).toBe('');
  });

  it('is case-insensitive for the scheme', () => {
    expect(wsUrlToHttp('WS://robot:5000/ws')).toBe('http://robot:5000/ws');
    expect(wsUrlToHttp('WSS://robot:5000/ws')).toBe('https://robot:5000/ws');
  });
});

// ─────────────────────────────────────────────────────────────
// toHealthUrl
// ─────────────────────────────────────────────────────────────

describe('toHealthUrl', () => {
  it('returns empty string for undefined input', () => {
    expect(toHealthUrl(undefined)).toBe('');
  });

  it('returns empty string for empty string input', () => {
    expect(toHealthUrl('')).toBe('');
  });

  // Scheme coercion — the function must NEVER return a ws:// or wss:// URL
  it('normalizes ws:// to http:// before returning', () => {
    const result = toHealthUrl('ws://robot:5000/video_feed');
    expect(result).toMatch(/^http:\/\//);
  });

  it('normalizes wss:// to https:// before returning', () => {
    const result = toHealthUrl('wss://robot:5000/video_feed');
    expect(result).toMatch(/^https:\/\//);
  });

  // Path transformation — /video_feed → /health
  it('replaces /video_feed with /health for ws:// upstream URL', () => {
    expect(toHealthUrl('ws://robot:5000/video_feed')).toBe('http://robot:5000/health');
  });

  it('replaces /video_feed with /health for http:// upstream URL', () => {
    expect(toHealthUrl('http://robot:5000/video_feed')).toBe('http://robot:5000/health');
  });

  it('replaces /video_feed with /health for wss:// upstream URL', () => {
    expect(toHealthUrl('wss://robot:5000/video_feed')).toBe('https://robot:5000/health');
  });

  // Proxy rewriting
  it('maps /api/video-proxy to /api/video-health (same-origin proxy)', () => {
    expect(toHealthUrl('/api/video-proxy?profile=lan')).toBe('/api/video-health?profile=lan');
  });

  it('maps /api/video-proxy (no query string) to /api/video-health', () => {
    expect(toHealthUrl('/api/video-proxy')).toBe('/api/video-health');
  });

  it('strips cache-buster "b" param from proxy URL', () => {
    expect(toHealthUrl('/api/video-proxy?profile=lan&b=12345')).toBe(
      '/api/video-health?profile=lan'
    );
  });

  it('strips cache-buster when it is the only param', () => {
    const result = toHealthUrl('/api/video-proxy?b=abc');
    expect(result).toBe('/api/video-health');
  });

  // Preserves port
  it('preserves the port number in the output URL', () => {
    expect(toHealthUrl('ws://omegaone:8080/video_feed')).toBe(
      'http://omegaone:8080/health'
    );
  });
});
