/*
# File: /Omega-Code/ui/src/utils/netProfile.ts
# Summary:
#   Profile-aware URL helpers for all services, powered by the resolver.
#   - No direct process.env reads in the client (Next-safe).
#   - Honors ?profile, MagicDNS, LAN→localhost fallback, https→wss, etc.
#   - Exposes both single best URL and full candidate lists per service.
#
#   Usage:
#     import { net } from '@/utils/netProfile';
#     const moveUrl = net.ws.movement();         // single best URL
#     const allMove = net.candidates.movement(); // ordered fallbacks
*/

'use client';

import {
  resolveWsUrl,
  resolveWsCandidates,
  getActiveProfile,
  type NetProfile,
} from './resolveWsUrl';

const DEBUG =
  (typeof window !== 'undefined' &&
    (process.env.NEXT_PUBLIC_WS_DEBUG === '1' ||
      (window as any).__ENV__?.NEXT_PUBLIC_WS_DEBUG === '1')) ||
  false;

const dlog = (...a: any[]) => DEBUG && console.info('[netProfile]', ...a);

// ---- Default ports/paths per service (kept in one place) ----
const DEF = {
  video:      { port: '5000', path: '/video_feed' },
  movement:   { port: '8081', path: '' },
  ultrasonic: { port: '8080', path: '/ultrasonic' },
  line:       { port: '8090', path: '/line-tracker' },
  lighting:   { port: '8082', path: '/lighting' },
  location:   { port: '8091', path: '/location' },
} as const;

export const net = {
  /** Active profile as the resolver sees it (url ?profile= override supported). */
  profile(): NetProfile {
    const p = getActiveProfile();
    dlog('active profile:', p);
    return p;
  },

  /** Video MJPEG HTTP URL (not WS) – single best. */
  video(): string {
    const url = resolveWsUrl('NEXT_PUBLIC_VIDEO_STREAM_URL', {
      defaultPort: DEF.video.port,
      path: DEF.video.path,
    });
    dlog('video url:', url);
    return url;
  },

  /** WebSocket URLs – single best per service. */
  ws: {
    movement(): string {
      const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT', {
        defaultPort: DEF.movement.port,
        path: DEF.movement.path,
      });
      dlog('movement url:', url);
      return url;
    },
    ultrasonic(): string {
      const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
        defaultPort: DEF.ultrasonic.port,
        path: DEF.ultrasonic.path,
      });
      dlog('ultrasonic url:', url);
      return url;
    },
    line(): string {
      const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER', {
        defaultPort: DEF.line.port,
        path: DEF.line.path,
      });
      dlog('line url:', url);
      return url;
    },
    lighting(): string {
      const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING', {
        defaultPort: DEF.lighting.port,
        path: DEF.lighting.path,
      });
      dlog('lighting url:', url);
      return url;
    },
    location(): string {
      const url = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION', {
        defaultPort: DEF.location.port,
        path: DEF.location.path,
      });
      dlog('location url:', url);
      return url;
    },
  },

  /** Ordered fallback candidates – useful for logs/diagnostics. */
  candidates: {
    movement(): string[] {
      const c = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT', {
        defaultPort: DEF.movement.port,
        path: DEF.movement.path,
      });
      dlog('movement candidates:', c);
      return c;
    },
    ultrasonic(): string[] {
      const c = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
        defaultPort: DEF.ultrasonic.port,
        path: DEF.ultrasonic.path,
      });
      dlog('ultrasonic candidates:', c);
      return c;
    },
    line(): string[] {
      const c = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER', {
        defaultPort: DEF.line.port,
        path: DEF.line.path,
      });
      dlog('line candidates:', c);
      return c;
    },
    lighting(): string[] {
      const c = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING', {
        defaultPort: DEF.lighting.port,
        path: DEF.lighting.path,
      });
      dlog('lighting candidates:', c);
      return c;
    },
    location(): string[] {
      const c = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION', {
        defaultPort: DEF.location.port,
        path: DEF.location.path,
      });
      dlog('location candidates:', c);
      return c;
    },
  },
};

export type { NetProfile };
