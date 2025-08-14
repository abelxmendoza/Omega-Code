// src/utils/resolveWsUrl.ts
'use client';

type NetProfile = 'LAN' | 'TAILSCALE' | 'LOCAL';
const PROFILE: NetProfile =
  (process.env.NEXT_PUBLIC_NETWORK_PROFILE?.toUpperCase() as NetProfile) || 'LOCAL';

const pick = (lan?: string, tailscale?: string, local?: string) =>
  PROFILE === 'LAN'       ? (lan ?? local ?? '') :
  PROFILE === 'TAILSCALE' ? (tailscale ?? local ?? '') :
                             (local ?? lan ?? tailscale ?? '');

/** All supported env bases (compile-time safe so Next inlines them). */
export type WsKey =
  | 'NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING'
  | 'NEXT_PUBLIC_BACKEND_WS_URL_LOCATION'
  | 'NEXT_PUBLIC_VIDEO_STREAM_URL';

/** Resolve a WS/HTTP URL from env by profile (lan | tailscale | local). */
export function resolveWsUrl(key: WsKey): string {
  let url = '';
  switch (key) {
    case 'NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT':
      url = pick(
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL
      );
      break;

    case 'NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC':
      url = pick(
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL
      );
      break;

    case 'NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER':
      url = pick(
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LOCAL
      );
      break;

    case 'NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING':
      url = pick(
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LAN,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_TAILSCALE,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL
      );
      break;

    case 'NEXT_PUBLIC_BACKEND_WS_URL_LOCATION':
      url = pick(
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LAN,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_TAILSCALE,
        process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_LOCAL
      );
      break;

    case 'NEXT_PUBLIC_VIDEO_STREAM_URL':
      url = pick(
        process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LAN,
        process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE,
        process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
      );
      break;
  }

  if (!url) {
    // Helpful dev hint without crashing
    console.warn(`[resolveWsUrl] Missing env for ${key} (profile=${PROFILE})`);
  }
  return url;
}
