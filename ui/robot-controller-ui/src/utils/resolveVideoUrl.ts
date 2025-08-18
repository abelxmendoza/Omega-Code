// File: src/utils/resolveVideoUrl.ts
'use client';
import { getActiveProfile } from './resolveWsUrl';

const read = (k: string): string | undefined =>
  (process.env as any)?.[k] ??
  (typeof window !== 'undefined' ? (window as any).__ENV__?.[k] : undefined);

function pageHttps() {
  return typeof window !== 'undefined' && location.protocol === 'https:';
}

/** Build candidates in priority order; allows ?video= override. */
export function resolveVideoCandidates(): string[] {
  const qp = typeof window !== 'undefined'
    ? new URLSearchParams(location.search).get('video') || undefined
    : undefined;

  const prof = getActiveProfile();
  const base =
    prof === 'tailscale' ? read('NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE') :
    prof === 'lan'       ? read('NEXT_PUBLIC_VIDEO_STREAM_URL_LAN') :
                           read('NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL');

  const others = [
    read('NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE'),
    read('NEXT_PUBLIC_VIDEO_STREAM_URL_LAN'),
    read('NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL'),
  ].filter(Boolean) as string[];

  const seen = new Set<string>();
  const list = [qp, base, ...others]
    .filter(Boolean)
    .filter((u) => (seen.has(u!) ? false : (seen.add(u!), true))) as string[];

  return list;
}

/** Best single video URL. */
export function resolveVideoUrl(): string {
  return resolveVideoCandidates()[0] ?? '';
}

/** Handy guard you can warn on in the UI. */
export function willBeMixedContent(url: string): boolean {
  if (!url) return false;
  if (!pageHttps()) return false;
  try {
    const u = new URL(url);
    return u.protocol === 'http:' && u.hostname !== location.hostname;
  } catch { return false; }
}
