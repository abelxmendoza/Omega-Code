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
#     the health latency badge may show “— ms” (that’s expected). The image onload/onerror still
#     drives status reliably, and the stream avoids CORS issues via the proxy.
#   • When NEXT_PUBLIC_MOCK_WS=1 is set, the page does NOT open a real WebSocket connection.
#     UI still functions using the mock autonomy wire and local logs.
*/

import Head from 'next/head';
import dynamic from 'next/dynamic';
import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import SpeedControl from '../components/control/SpeedControl';
import MotorTelemetryPanel from '../components/control/MotorTelemetryPanel';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/sensors/SensorDashboard';
import ErrorBoundary from '../components/ErrorBoundary';
import CarControlPanel from '../components/control/CarControlPanel';
import CameraControlPanel from '../components/control/CameraControlPanel';
import ServoTelemetryPanel from '../components/control/ServoTelemetryPanel';
import { robotWS } from '../utils/network';
import PerformanceDashboard from '../components/PerformanceDashboard';
import EnhancedServoTelemetryPanel from '../components/control/EnhancedServoTelemetryPanel';
import AutonomyPanel from '../components/control/AutonomyModal';
import SystemModeDashboard from '../components/SystemModeDashboard';
import VisionModePanel from '../components/VisionModePanel';
import LatencyDashboard from '../components/LatencyDashboard';
import { useCommand } from '../context/CommandContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import LedModal from '../components/lighting/LedModal';
import { v4 as uuidv4 } from 'uuid';
import { useGamepad } from '../hooks/useGamepad';
import XboxControllerStatus from '../components/control/XboxControllerStatus';

// Autonomy API client (WS/HTTP/mock wires)
import {
  makeAutonomyApi,
  createWsWire,
  mockWire,
} from '@/utils/autonomyApi';

const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG;
const MOCK_WS = process.env.NEXT_PUBLIC_MOCK_WS === '1';

/* ------------------ Client-only camera (avoids SSR issues) ------------------ */
const CameraFrame = dynamic(() => import('../components/CameraFrame'), {
  ssr: false,
  loading: () => (
    <div className="w-full">
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
const nextBackoff = (prev: number) => Math.min(30_000, Math.max(500, Math.floor(prev * 1.6)));

/* ---------------------- resolved endpoints (by profile) ---------------------- */

/** Movement control WS resolved from env (lan|tailscale|local). */
const movementWsResolved = getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');

/* --------------------------------- Page --------------------------------- */

export default function Home() {
  const { addCommand } = useCommand();

  const [isLedModalOpen, setIsLedModalOpen] = useState(false);
  const [gamepadPaused, setGamepadPaused] = useState(false);

  /* -------- Camera proxy URL (built on client so we can read page query) -------- */
  const cameraProxyUrl = useMemo(() => {
    const prof = getActiveProfile();
    const qs = new URLSearchParams();
    qs.set('profile', prof);

    if (typeof window !== 'undefined') {
      const url = new URL(window.location.href);
      const override = url.searchParams.get('video');
      const qProf = url.searchParams.get('profile');
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
  const keepAliveTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const backoffRef = useRef<number>(800);
  const closingRef = useRef<boolean>(false);
  const wsConnectedOnce = useRef<boolean>(false);
  const wsUrlMissingLogged = useRef<boolean>(false);

  const connectWebSocket = useCallback(() => {
    const rawUrl = movementWsResolved;
    if (!rawUrl) {
      if (!wsUrlMissingLogged.current) {
        wsUrlMissingLogged.current = true;
        addCommand('WS URL missing: set NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_* in your .env.local');
      }
      return;
    }
    const url = coerceWsScheme(rawUrl);

    try {
      if (DEBUG) console.log('[WS] connecting →', url);
      // Use robotWS wrapper to respect offline mode
      ws.current = robotWS(url);
      if (!ws.current) {
        addCommand('Robot backend offline — WebSocket disabled');
        return;
      }

      ws.current.onopen = () => {
        wsConnectedOnce.current = true;
        addCommand('WebSocket connection established');
        if (DEBUG) console.log('[WS] open');
        backoffRef.current = 800;

        if (keepAliveTimer.current) clearInterval(keepAliveTimer.current);
        keepAliveTimer.current = setInterval(() => {
          try {
            if (ws.current?.readyState === WebSocket.OPEN) {
              ws.current.send(JSON.stringify({ type: 'keep-alive', ts: Date.now() }));
            }
          } catch {
            /* swallow */
          }
        }, 15_000); // Reduced from 25s to 15s for mobile hotspots
      };

      ws.current.onclose = (ev) => {
        if (DEBUG) console.log('[WS] close', ev?.reason || '');
        if (keepAliveTimer.current) { clearInterval(keepAliveTimer.current); keepAliveTimer.current = null; }
        if (closingRef.current) return; // user-initiated close during unmount

        // Only surface a log entry when we had a real connection; silent retries otherwise
        if (wsConnectedOnce.current) {
          wsConnectedOnce.current = false;
          addCommand('WebSocket connection lost. Reconnecting…');
        }
        if (reconnectTimer.current) { clearTimeout(reconnectTimer.current); reconnectTimer.current = null; }
        const delay = Math.min(backoffRef.current, 30000);
        reconnectTimer.current = setTimeout(() => {
          backoffRef.current = nextBackoff(backoffRef.current);
          connectWebSocket();
        }, delay);
      };

      ws.current.onerror = (error) => {
        if (DEBUG) console.warn('[WS] error', error);
        // onclose handler will schedule reconnect
      };

      ws.current.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message?.command) addCommand(`Received: ${message.command}`);
        } catch {
          addCommand('WS message parse error');
        }
      };
    } catch (err) {
      if (DEBUG) console.warn('[WS] connect exception:', String(err));
      const delay = backoffRef.current;
      reconnectTimer.current = setTimeout(() => {
        backoffRef.current = nextBackoff(backoffRef.current);
        connectWebSocket();
      }, delay);
    }
  }, [addCommand]);

  // Only open a real socket if mocks are OFF.
  useEffect(() => {
    if (MOCK_WS) {
      if (DEBUG) console.log('[WS] MOCK_WS=1 → skipping real WebSocket connect');
      addCommand('Mock WS mode: UI will not open real sockets.');
      return; // no connect
    }

    connectWebSocket();
    return () => {
      // graceful shutdown
      closingRef.current = true;
      if (keepAliveTimer.current) { clearInterval(keepAliveTimer.current); keepAliveTimer.current = null; }
      if (reconnectTimer.current) { clearTimeout(reconnectTimer.current); reconnectTimer.current = null; }
      try { ws.current?.close(); } catch {}
      ws.current = null;
    };
  }, [connectWebSocket, addCommand]);

  /* ----------------------------- Gamepad ---------------------------------- */
  // Pass the ws ref directly so the hook always reads the live socket even
  // after reconnects, without triggering re-renders.
  const gamepadState = useGamepad({ wsRef: ws, paused: gamepadPaused || MOCK_WS });

  /* ---------------- Autonomy API (WS wire, with mock toggle) --------------- */
  const autonomyApi = useMemo(() => {
    try {
      if (MOCK_WS || !movementWsResolved) {
        return makeAutonomyApi(mockWire);
      }
      const wire = createWsWire(() => coerceWsScheme(movementWsResolved), { timeoutMs: 3000 });
      return makeAutonomyApi(wire);
    } catch (e) {
      return makeAutonomyApi(mockWire);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [movementWsResolved]);

  // Log autonomy API status after render
  useEffect(() => {
    if (MOCK_WS || !movementWsResolved) {
      addCommand('Autonomy API using MOCK wire.');
    }
  }, [addCommand]);

  /* ---------------------- Send command helper with logs --------------------- */
  const sendCommandWithLog = useCallback(
    (command: string, angle: number = 0) => {
      const requestId = uuidv4();

      // In mock mode, don't send—just log.
      if (MOCK_WS) {
        addCommand(`[MOCK] Sent: ${command} (ID: ${requestId})`);
        return;
      }

      if (ws.current?.readyState === WebSocket.OPEN) {
        try {
          ws.current.send(JSON.stringify({ command, angle, request_id: requestId }));
          addCommand(`Sent: ${command} (ID: ${requestId})`);
        } catch (e) {
          addCommand(`Failed to send command: ${String(e)}`);
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
        case 'p': event.preventDefault(); return sendCommandWithLog(COMMAND.INCREASE_SPEED);
        case 'o': event.preventDefault(); return sendCommandWithLog(COMMAND.DECREASE_SPEED);
        case ' ': event.preventDefault(); return sendCommandWithLog(COMMAND.CMD_BUZZER);
        case 'i': event.preventDefault(); return setIsLedModalOpen(true);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [sendCommandWithLog]);

  /* ---------------------------------- UI ---------------------------------- */
  const isConnected =
    MOCK_WS || (ws.current?.readyState === WebSocket.OPEN);

  return (
    <div className="cyber-theme min-h-screen overflow-x-hidden">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
        <style jsx global>{`
          html, body {
            overflow-x: hidden;
            max-width: 100vw;
          }
        `}</style>
      </Head>

      <Header
        batteryLevel={75}
        gamepadConnected={gamepadState.connected}
        gamepadName={gamepadState.name}
        gamepadPaused={gamepadPaused}
      />

      <main className="cyber-content px-4 xl:px-8 py-4 overflow-x-hidden">
        <div className="max-w-[1600px] mx-auto space-y-6">

          {/* Zone 1: Controls | Camera | Camera Controls */}
          <div className="grid grid-cols-1 xl:grid-cols-[auto_1fr_auto] gap-4 xl:gap-6 items-center">
            <div className="shrink-0 space-y-3">
              <CarControlPanel />
              <AutonomyPanel
                connected={isConnected}
                batteryPct={75}
                onStart={async (mode, params) => {
                  try {
                    await autonomyApi.start(mode, params);
                    addCommand(`Sent: autonomy-start (${mode})`);
                    autonomyApi.maybeSuggestLights(Boolean((params as any)?.headlights));
                  } catch (e) {
                    addCommand(`autonomy-start failed: ${String(e)}`);
                  }
                }}
                onStop={async () => {
                  try {
                    await autonomyApi.stop();
                    addCommand('Sent: autonomy-stop');
                  } catch (e) {
                    addCommand(`autonomy-stop failed: ${String(e)}`);
                  }
                }}
                onDock={async () => {
                  try {
                    await autonomyApi.dock();
                    addCommand('Sent: autonomy-dock');
                  } catch (e) {
                    addCommand(`autonomy-dock failed: ${String(e)}`);
                  }
                }}
                onSetWaypoint={async (label, lat, lon) => {
                  try {
                    await autonomyApi.setWaypoint(label, lat, lon);
                    addCommand(`Sent: set-waypoint "${label}" (${lat}, ${lon})`);
                  } catch (e) {
                    addCommand(`set-waypoint failed: ${String(e)}`);
                  }
                }}
                onUpdate={async (params) => {
                  try {
                    await autonomyApi.update(params);
                    addCommand('Sent: autonomy-update (params)');
                  } catch (e) {
                    addCommand(`autonomy-update failed: ${String(e)}`);
                  }
                }}
              />
              <XboxControllerStatus
                state={gamepadState}
                paused={gamepadPaused}
                onTogglePause={() => setGamepadPaused(p => !p)}
              />
            </div>

            {/* Camera fills available space */}
            <div className="min-w-0">
              <CameraFrame
                src={cameraProxyUrl}
                title="Front Camera"
                className="w-full"
              />
              <VisionModePanel />
            </div>

            <div className="shrink-0">
              <CameraControlPanel />
            </div>
          </div>

          {/* Zone 2: Sensors | Telemetry */}
          <div className="grid grid-cols-1 xl:grid-cols-[520px_1fr] gap-4 xl:gap-6 items-start">
            <ErrorBoundary>
              <SensorDashboard />
            </ErrorBoundary>
            <div className="grid grid-cols-1 xl:grid-cols-3 gap-4">
              <SpeedControl />
              <MotorTelemetryPanel />
              <EnhancedServoTelemetryPanel />
            </div>
          </div>

          {/* Zone 3: System Mode + Latency side by side */}
          <div className="grid grid-cols-1 xl:grid-cols-2 gap-4 xl:gap-6">
            <ErrorBoundary>
              <SystemModeDashboard />
            </ErrorBoundary>
            <ErrorBoundary>
              <LatencyDashboard />
            </ErrorBoundary>
          </div>

          {/* Zone 4: Performance Dashboard (full width below) */}
          <ErrorBoundary>
            <div className="flex justify-center">
              <PerformanceDashboard />
            </div>
          </ErrorBoundary>

          <div className="flex justify-center">
            <CommandLog />
          </div>

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
