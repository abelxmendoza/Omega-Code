/*
# File: /src/pages/index.tsx
# Summary:
#   Main entry for the robot controller UI.
#   - Client-only CameraFrame (avoids SSR crashes)
#   - Robot controls, logs, sensors, lighting modal
#   - Profile-based endpoint selection (lan | tailscale | local)
#   - WebSocket keep-alive + exponential backoff reconnect
#   - Hotkeys (P/O/Space/I)
*/

import Head from 'next/head';
import dynamic from 'next/dynamic';
import React, { useState, useEffect, useRef, useCallback } from 'react';
import SpeedControl from '../components/control/SpeedControl';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/sensors/SensorDashboard';
import CarControlPanel from '../components/control/CarControlPanel';
import CameraControlPanel from '../components/control/CameraControlPanel';
import { useCommand } from '../context/CommandContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import LedModal from '../components/lighting/LedModal';
import { v4 as uuidv4 } from 'uuid';
// If you already have a util for active profile, you can import it:
// import { getActiveProfile } from '@/utils/resolveWsUrl';

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

/** Read env var by profile, with LOCAL fallback. Example base: 'NEXT_PUBLIC_VIDEO_STREAM_URL'. */
const getEnvByProfile = (base: string, profile?: string): string => {
  const p = (profile || getActiveProfile()).toUpperCase();
  return (
    process.env[`${base}_${p}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
};

/** Optional host fallback like omega1-1.hartley-ghost.ts.net */
const getRobotHostFallback = (): string => {
  const prof = getActiveProfile().toUpperCase();
  return (
    process.env[`NEXT_PUBLIC_ROBOT_HOST_${prof}`] ||
    process.env['NEXT_PUBLIC_ROBOT_HOST_LOCAL'] ||
    ''
  );
};

/** Build a sensible default video URL if env is missing. */
const defaultVideoUrl = (): string => {
  const prof = getActiveProfile();
  const host = getRobotHostFallback();
  if (!host) return ''; // no good fallback
  // direct MJPEG feed path on backend
  return `http://${host}:5000/video_feed`;
};

/** Honor NEXT_PUBLIC_WS_FORCE_INSECURE=1 to keep ws:// even on https pages. */
const coerceWsScheme = (rawUrl: string): string => {
  if (!rawUrl) return rawUrl;
  try {
    const u = new URL(rawUrl, window.location.href);
    const forceInsecure = !!process.env.NEXT_PUBLIC_WS_FORCE_INSECURE;
    if (forceInsecure) {
      if (u.protocol === 'wss:') u.protocol = 'ws:';
      return u.toString();
    }
    // Otherwise: if page is https and url is ws://, upgrade to wss://
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

// Direct video stream (we pass direct URL to CameraFrame so its /health derivation works)
const videoUrlResolved =
  getEnvByProfile('NEXT_PUBLIC_VIDEO_STREAM_URL') || defaultVideoUrl();

// Movement control WS (envs you already have in .env.local)
//   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_TAILSCALE
//   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LAN
//   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL
const movementWsResolved = getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');

/* --------------------------------- Page --------------------------------- */

export default function Home() {
  const { addCommand } = useCommand();

  const [isLedModalOpen, setIsLedModalOpen] = useState(false);

  const ws = useRef<WebSocket | null>(null);
  const keepAliveTimer = useRef<number | null>(null);
  const reconnectTimer = useRef<number | null>(null);
  const backoffRef = useRef<number>(800);

  /* --------------------------- WebSocket connect --------------------------- */
  const connectWebSocket = useCallback(() => {
    // Pick final WS URL with scheme fixes
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

        // reset backoff on success
        backoffRef.current = 800;

        // keep-alive ping
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
        addCommand(`WebSocket error`);
        // Let onclose handle the reconnect/backoff; most browsers call close after error.
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
      // schedule reconnect
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
            <CarControlPanel sendCommand={sendCommandWithLog} />
          </div>

          {/* Framed camera */}
          <div className="flex-shrink-0">
            <CameraFrame
              src={videoUrlResolved}
              title="Front Camera"
              className="w-[720px] max-w-[42vw]"
            />
          </div>

          <div className="flex-shrink-0">
            <CameraControlPanel sendCommand={sendCommandWithLog} />
          </div>
        </div>

        <div className="flex justify-center items-center space-x-6 mt-6">
          <div className="w-1/3">
            <SensorDashboard />
          </div>
          <div className="w-1/4">
            <SpeedControl
              sendCommand={sendCommandWithLog}
              onOpenLedModal={() => setIsLedModalOpen(true)}
            />
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
