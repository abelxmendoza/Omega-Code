/*
# File: /src/pages/index.tsx
# Summary:
#   Main entry for the robot controller UI.
#   - Client-only CameraFrame (avoids SSR crashes)
#   - Robot controls, logs, sensors, lighting modal
#   - Profile-based endpoint selection (lan | tailscale | local)
#   - WebSocket keep-alive + exponential backoff reconnect
#   - Hotkeys (P/O/Space/I)
#   - Camera wired through same-origin proxy (/api/video-proxy) to avoid mixed content/CORS
#
#   Notes:
#   • Camera health ping inside CameraFrame derives /health from the src. Since we feed the proxy,
#     the health latency badge will show “— ms” (that’s expected). The image onload/onerror still
#     drives status reliably, and the stream avoids CORS issues via the proxy.
*/

import Head from 'next/head';
import dynamic from 'next/dynamic';
import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import SpeedControl from '../components/control/SpeedControl';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/sensors/SensorDashboard';
import CarControlPanel from '../components/control/CarControlPanel';
import CameraControlPanel from '../components/control/CameraControlPanel';
import AutonomyPanel from '../components/control/AutonomyModal'; // ← NEW
import { useCommand } from '../context/CommandContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import LedModal from '../components/lighting/LedModal';
import { v4 as uuidv4 } from 'uuid';

const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG;

/* ------------------ Client-only camera (avoids SSR issues) ------------------ */
const CameraFrame = dynamic(() => import('../components/CameraFrame'), {
  ssr: false,
  loading: () => (
    <div className="w-[720px] max-w-[42vw]">
      <div
        className="relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden"
        style={{ paddingTop: '56.25%' }}
      >
        <div className="absolute inset-0 flex items-center justify-center text-white/70 text-sm">
          Loading camera…
        </div>
      </div>
    </div>
  ),
});

/* ----------------------------- helpers/env ----------------------------- */

/** Active network profile (lan | tailscale | local). */
const getActiveProfile = (): 'lan' | 'tailscale' | 'local' => {
  const p = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'].includes(p) ? p : 'local') as any;
};

/** Read env var by profile, with LOCAL fallback. Example base: 'NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'. */
const getEnvByProfile = (base: string, profile?: string): string => {
  const p = (profile || getActiveProfile()).toUpperCase();
  return (
    process.env[`${base}_${p}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
};

/** Optional host fallback like omega1-1.hartley-ghost.ts.net (useful if you ever need direct URLs). */
const getRobotHostFallback = (): string => {
  const prof = getActiveProfile().toUpperCase();
  return (
    process.env[`NEXT_PUBLIC_ROBOT_HOST_${prof}`] ||
    process.env['NEXT_PUBLIC_ROBOT_HOST_LOCAL'] ||
    ''
  );
};

/** Honor NEXT_PUBLIC_WS_FORCE_INSECURE=1 to keep ws:// even on https pages. */
const coerceWsScheme = (rawUrl: string): string => {
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
};

/** Exponential backoff helper (ms). */
const nextBackoff = (prev: number) => Math.min(10_000, Math.max(500, Math.floor(prev * 1.6)));

/* ---------------------- resolved endpoints (by profile) ---------------------- */

/** Movement control WS resolved from env (lan|tailscale|local). */
const movementWsResolved = getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');

/* --------------------------------- Page --------------------------------- */

export default function Home() {
  const { addCommand } = useCommand();

  const [isLedModalOpen, setIsLedModalOpen] = useState(false);

  /* -------- Camera proxy URL (built on client so we can read page query) -------- */
  const cameraProxyUrl = useMemo(() => {
    // Build /api/video-proxy?profile=<active>[&video=...]
    const prof = getActiveProfile();
    const qs = new URLSearchParams();
    qs.set('profile', prof);

    // honor page override ?video=… to force upstream
    if (typeof window !== 'undefined') {
      const url = new URL(window.location.href);
      const override = url.searchParams.get('video');
      const qProf = url.searchParams.get('profile'); // optional override for profile
      if (override) qs.set('video', override);
      if (qProf && ['lan', 'tailscale', 'local'].includes(qProf)) {
        qs.set('profile', qProf);
      }
    }

    const u = `/api/video-proxy?${qs.toString()}`;
    if (DEBUG) console.log('[netProfile] camera proxy url:', u);
    return u;
  }, []);

  /* --------------------------- WebSocket connect --------------------------- */
  const ws = useRef<WebSocket | null>(null);
  const keepAliveTimer = useRef<number | null>(null);
  const reconnectTimer = useRef<number | null>(null);
  const backoffRef = useRef<number>(800);

  const connectWebSocket = useCallback(() => {
    const rawUrl = movementWsResolved;
    if (!rawUrl) {
      addCommand('WS URL missing: set NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_* in your .env.local');
      return;
    }
    const url = coerceWsScheme(rawUrl);

    try {
      if (DEBUG) console.log('[WS] connecting →', url);
      ws.current = new WebSocket(url);

      ws.current.onopen = () => {
        addCommand('WebSocket connection established');
        if (DEBUG) console.log('[WS] open');

        backoffRef.current = 800;

        if (keepAliveTimer.current) window.clearInterval(keepAliveTimer.current);
        keepAliveTimer.current = window.setInterval(() => {
          try {
            if (ws.current?.readyState === WebSocket.OPEN) {
              ws.current.send(JSON.stringify({ type: 'keep-alive', ts: Date.now() }));
            }
          } catch {}
        }, 25_000);
      };

      ws.current.onclose = () => {
        if (DEBUG) console.log('[WS] close');
        addCommand('WebSocket connection closed. Reconnecting…');
        if (keepAliveTimer.current) { window.clearInterval(keepAliveTimer.current); keepAliveTimer.current = null; }
        if (reconnectTimer.current) { window.clearTimeout(reconnectTimer.current); reconnectTimer.current = null; }
        const delay = backoffRef.current;
        reconnectTimer.current = window.setTimeout(() => {
          backoffRef.current = nextBackoff(backoffRef.current);
          connectWebSocket();
        }, delay);
      };

      ws.current.onerror = (error) => {
        if (DEBUG) console.warn('[WS] error', error);
        addCommand('WebSocket error');
        // onclose will handle the backoff reconnect
      };

      ws.current.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.command) addCommand(`Received: ${message.command}`);
        } catch {
          addCommand('Error parsing WebSocket message');
        }
      };
    } catch (err) {
      addCommand(`WebSocket connect failed: ${String(err)}`);
      const delay = backoffRef.current;
      reconnectTimer.current = window.setTimeout(() => {
        backoffRef.current = nextBackoff(backoffRef.current);
        connectWebSocket();
      }, delay);
    }
  }, [addCommand]);

  useEffect(() => {
    connectWebSocket();
    return () => {
      if (keepAliveTimer.current) window.clearInterval(keepAliveTimer.current);
      if (reconnectTimer.current) window.clearTimeout(reconnectTimer.current);
      try { ws.current?.close(); } catch {}
    };
  }, [connectWebSocket]);

  /* ---------------------- Send command helper with logs --------------------- */
  const sendCommandWithLog = useCallback(
    (command: string, angle: number = 0) => {
      const requestId = uuidv4();
      if (ws.current?.readyState === WebSocket.OPEN) {
        try {
          ws.current.send(JSON.stringify({ command, angle, request_id: requestId }));
          addCommand(`Sent: ${command} (ID: ${requestId})`);
        } catch {
          addCommand('Failed to send command (serialization or socket error)');
        }
      } else {
        addCommand('Failed to send command: WebSocket is not open');
      }
    },
    [addCommand]
  );

  /* --------------------------------- Hotkeys -------------------------------- */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p': return sendCommandWithLog(COMMAND.INCREASE_SPEED);
        case 'o': return sendCommandWithLog(COMMAND.DECREASE_SPEED);
        case ' ': return sendCommandWithLog(COMMAND.CMD_BUZZER);
        case 'i': return setIsLedModalOpen(true);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [sendCommandWithLog]);

  /* ---------------------------------- UI ---------------------------------- */
  return (
    <div className="min-h-screen bg-gray-50">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      {/* Header computes live service dots internally; passing battery for now */}
      <Header batteryLevel={75} />

      <main className="p-4 space-y-4">
        <div className="flex justify-center items-center space-x-8">
          <div className="flex-shrink-0">
            {/* CarControlPanel likely already uses CommandContext, so no prop needed */}
            <CarControlPanel />
            {/* ↓ Autonomy panel sits right under the car controller */}
            <AutonomyPanel
              connected={ws.current?.readyState === WebSocket.OPEN}
              batteryPct={75}
              onStart={(mode, params) => {
                if (ws.current?.readyState === WebSocket.OPEN) {
                  ws.current.send(JSON.stringify({ command: 'autonomy-start', mode, params }));
                  addCommand(`Sent: autonomy-start (${mode})`);
                } else {
                  addCommand('autonomy-start failed: WS not open');
                }
              }}
              onStop={() => {
                if (ws.current?.readyState === WebSocket.OPEN) {
                  ws.current.send(JSON.stringify({ command: 'autonomy-stop' }));
                  addCommand('Sent: autonomy-stop');
                } else {
                  addCommand('autonomy-stop failed: WS not open');
                }
              }}
              onDock={() => {
                if (ws.current?.readyState === WebSocket.OPEN) {
                  ws.current.send(JSON.stringify({ command: 'autonomy-dock' }));
                  addCommand('Sent: autonomy-dock');
                } else {
                  addCommand('autonomy-dock failed: WS not open');
                }
              }}
              onSetWaypoint={(label, lat, lon) => {
                if (ws.current?.readyState === WebSocket.OPEN) {
                  ws.current.send(JSON.stringify({ command: 'set-waypoint', label, lat, lon }));
                  addCommand(`Sent: set-waypoint "${label}" (${lat}, ${lon})`);
                } else {
                  addCommand('set-waypoint failed: WS not open');
                }
              }}
            />
          </div>

          {/* Framed camera (via proxy to avoid mixed content/CORS) */}
          <div className="flex-shrink-0">
            <CameraFrame
              src={cameraProxyUrl}
              title="Front Camera"
              className="w-[720px] max-w-[42vw]"
            />
          </div>

          <div className="flex-shrink-0">
            {/* CameraControlPanel uses context; don't pass sendCommand */}
            <CameraControlPanel />
          </div>
        </div>

        <div className="flex justify-center items-center space-x-6 mt-6">
          <div className="w-1/3">
            <SensorDashboard />
          </div>
          <div className="w-1/4">
            {/* SpeedControl uses context; no props required */}
            <SpeedControl />
            {/* Local button to open LEDs modal (replaces onOpenLedModal prop) */}
            <button
              className="mt-3 w-full bg-green-600 hover:bg-green-700 text-white text-sm px-3 py-2 rounded-md"
              onClick={() => setIsLedModalOpen(true)}
              type="button"
            >
              LEDs…
            </button>
          </div>
        </div>

        <div className="flex flex-col items-center space-y-4 mt-4">
          <CommandLog />
        </div>
      </main>

      {isLedModalOpen && (
        <LedModal
          isOpen={isLedModalOpen}
          onClose={() => setIsLedModalOpen(false)}
        />
      )}
    </div>
  );
}

