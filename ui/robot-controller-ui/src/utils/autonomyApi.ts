/*
File: /src/utils/autonomyApi.ts
Summary:
  Tiny client for autonomy commands. Works with either:
    • WebSocket wire (send JSON frames: { command: 'autonomy-*', ... })
    • HTTP wire      (POST JSON to <base>/<topic>)
    • Mock wire      (logs to console; great for dev)

  Exposes:
    makeAutonomyApi(wire) → { start, stop, update, dock, setWaypoint, maybeSuggestLights }

  Example:
    import { makeAutonomyApi, createWsWire } from '@/utils/autonomyApi';
    const wire = createWsWire(() => movementWsResolved, { timeoutMs: 3000 });
    const api  = makeAutonomyApi(wire);
    await api.start('line_follow', { speedPct: 40, obstacleAvoidance: true });

Notes:
  - No top-level WebSocket usage (safe to import on server).
  - Topics like 'autonomy/start' become 'autonomy-start' in frames.
  - Throws on failure so the UI can toast/log errors.
*/

export type AutonomyMode =
  | 'idle' | 'patrol' | 'follow' | 'dock' | 'line_track'
  | 'line_follow' | 'avoid_obstacles' | 'edge_detect' | 'waypoints'
  | 'color_track' | 'aruco' | 'person_follow' | 'scan_servo';

// Keep params open-ended; UI can pass any shape your backend expects.
export type AutonomyParams = Record<string, unknown>;

export type AutonomyWire = {
  /** Send a message on a topic with an optional payload. May throw on failure. */
  send: (topic: string, payload?: unknown) => Promise<void> | void;
};

/* --------------------------------------------------------------------------------
 * Factory: high-level API your UI calls
 * -------------------------------------------------------------------------------*/
export function makeAutonomyApi(wire: AutonomyWire) {
  return {
    start(mode: AutonomyMode, params: AutonomyParams) {
      const canonical = canonicalizeMode(mode);
      return wire.send('autonomy/start', { mode: canonical, params });
    },
    stop() {
      return wire.send('autonomy/stop');
    },
    update(params: AutonomyParams) {
      return wire.send('autonomy/update', { params });
    },
    dock() {
      return wire.send('autonomy/dock');
    },
    setWaypoint(label: string, lat: number, lon: number) {
      return wire.send('autonomy/set_waypoint', { label, lat, lon });
    },
    /** Optional helper: hint your LED system in autonomy. */
    maybeSuggestLights(enabled: boolean) {
      if (enabled) {
        try { wire.send('lighting/preset', { preset: 'autonomy' }); } catch { /* best-effort */ }
      }
    },
  };
}

/* --------------------------------------------------------------------------------
 * WIRE #1: WebSocket adapter (UI/browser side)
 *   - Lazily opens the socket using a URL (string or fn) the first time you send.
 *   - Waits until OPEN (with timeout) then sends.
 *   - Frame: { command: 'autonomy-*', ...payload }
 * -------------------------------------------------------------------------------*/
export function createWsWire(
  url: string | (() => string),
  opts: { timeoutMs?: number } = {}
): AutonomyWire {
  let ws: WebSocket | null = null;
  let openPromise: Promise<void> | null = null;
  const timeoutMs = Math.max(500, opts.timeoutMs ?? 2500);

  function getUrl(): string {
    return typeof url === 'function' ? url() : url;
  }

  function ensureOpen(): Promise<void> {
    if (typeof window === 'undefined') {
      throw new Error('WebSocket wire can only run in the browser.');
    }
    if (ws && ws.readyState === WebSocket.OPEN) return Promise.resolve();

    if (!openPromise) {
      openPromise = new Promise<void>((resolve, reject) => {
        try {
          ws = new WebSocket(getUrl());
        } catch (e) {
          openPromise = null;
          reject(new Error(`WS create failed: ${String(e)}`));
          return;
        }

        const timer = setTimeout(() => {
          cleanup();
          reject(new Error(`WS open timeout after ${timeoutMs}ms`));
        }, timeoutMs);

        function cleanup() {
          clearTimeout(timer);
          if (ws) {
            ws.onopen = null;
            ws.onclose = null;
            ws.onerror = null;
          }
          openPromise = null;
        }

        ws.onopen = () => { cleanup(); resolve(); };
        ws.onclose = () => { cleanup(); reject(new Error('WS closed during connect')); };
        ws.onerror = () => { cleanup(); reject(new Error('WS error during connect')); };
      });
    }

    return openPromise;
  }

  return {
    async send(topic: string, payload?: unknown) {
      await ensureOpen();

      if (!ws || ws.readyState !== WebSocket.OPEN) {
        throw new Error('WS not open');
      }

      const command = topic.replace(/\//g, '-');

      // Build a safe body that never spreads undefined/null
      const body =
        payload && typeof payload === 'object'
          ? { command, ...(payload as Record<string, unknown>) }
          : payload === undefined
            ? { command }
            : { command, payload };

      try {
        ws.send(JSON.stringify(body));
      } catch (e) {
        throw new Error(`WS send failed: ${String(e)}`);
      }
    },
  };
}

/* --------------------------------------------------------------------------------
 * WIRE #2: HTTP adapter (if you prefer POST endpoints)
 *   - POSTs JSON to `${base}/${topic}` (slashes kept in path).
 *   - Body: { command: 'autonomy-*', ...payload }
 * -------------------------------------------------------------------------------*/
export function createHttpWire(baseUrl: string, opts: RequestInit = {}): AutonomyWire {
  const base = baseUrl.replace(/\/+$/, ''); // strip trailing slash

  return {
    async send(topic: string, payload?: unknown) {
      const url = `${base}/${topic}`;
      const command = topic.replace(/\//g, '-');

      const body =
        payload && typeof payload === 'object'
          ? { command, ...(payload as Record<string, unknown>) }
          : payload === undefined
            ? { command }
            : { command, payload };

      let res: Response;
      try {
        res = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json', ...(opts.headers || {}) },
          body: JSON.stringify(body),
          ...opts,
        });
      } catch (e) {
        throw new Error(`POST ${url} failed to send: ${String(e)}`);
      }

      if (!res.ok) {
        let msg = `HTTP ${res.status}`;
        try { msg = (await res.text()) || msg; } catch {}
        throw new Error(`POST ${url} failed: ${msg}`);
      }
    },
  };
}

/* --------------------------------------------------------------------------------
 * WIRE #3: Mock adapter (dev/demo when robot is offline)
 * -------------------------------------------------------------------------------*/
export const mockWire: AutonomyWire = {
  async send(topic, payload) {
    // eslint-disable-next-line no-console
    console.log('[MOCK SEND]', topic, payload);
    await delay(150);
  },
};

/* --------------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------------*/
function canonicalizeMode(m: AutonomyMode): AutonomyMode {
  return m === 'line_track' ? 'line_follow' : m;
}

function delay(ms: number) {
  return new Promise<void>(r => setTimeout(r, ms));
}
