/*
# File: /src/pages/index.tsx
# Summary:
#   Main entry for the robot controller UI.
#   - Client-only CameraFrame (avoids SSR crashes)
#   - Robot controls, logs, sensors, lighting modal
#   - Profile-based endpoint selection (lan | tailscale | local)
#   - Hotkeys (P/O/Space/I)
#   - Camera wired through same-origin proxy (/api/video-proxy) to avoid mixed content/CORS
#
#   Movement WebSocket is managed exclusively by CommandContext — this page does NOT
#   open its own WS connection. All command sending goes through CommandContext.sendCommand().
#
#   Notes:
#   • Camera health ping inside CameraFrame derives /health from the src. Since we feed the proxy,
#     the health latency badge may show “— ms” (that’s expected). The image onload/onerror still
#     drives status reliably, and the stream avoids CORS issues via the proxy.
#   • When NEXT_PUBLIC_MOCK_WS=1 is set, the autonomy API falls back to a mock wire.
*/

import Head from 'next/head';
import dynamic from 'next/dynamic';
import React, { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import SpeedControl from '../components/control/SpeedControl';
import MotorTelemetryPanel from '../components/control/MotorTelemetryPanel';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/sensors/SensorDashboard';
import ErrorBoundary from '../components/ErrorBoundary';
import CarControlPanel from '../components/control/CarControlPanel';
import CameraControlPanel from '../components/control/CameraControlPanel';
import ServoTelemetryPanel from '../components/control/ServoTelemetryPanel';
import PerformanceDashboard from '../components/PerformanceDashboard';
import EnhancedServoTelemetryPanel from '../components/control/EnhancedServoTelemetryPanel';
import AutonomyPanel from '../components/control/AutonomyModal';
import SystemStatusPanel from '../components/SystemStatusPanel';
import VisionModePanel from '../components/VisionModePanel';
import LatencyDashboard from '../components/LatencyDashboard';
import { useCommand } from '../context/CommandContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import LedModal from '../components/lighting/LedModal';
import { useGamepad } from '../hooks/useGamepad';
import { usePiGamepad } from '../hooks/usePiGamepad';
import XboxControllerStatus from '../components/control/XboxControllerStatus';
import LocalizationPanel from '../components/LocalizationPanel';
import DemoModeToggle from '../components/DemoModeToggle';
import ConnectionModeSelector from '../components/ConnectionModeSelector';

// Autonomy API client (HTTP wire → FastAPI /autonomy/* endpoints)
import {
  makeAutonomyApi,
  createHttpWire,
} from '@/utils/autonomyApi';
import { getActiveProfile } from '@/utils/envProfile';
import { useDemoMode } from '@/context/DemoModeContext';

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

/* --------------------------------- Page --------------------------------- */

export default function Home() {
  const { addCommand, sendCommand, status } = useCommand();
  const { demoMode, engine: simEngine } = useDemoMode();

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

  /* Movement WebSocket is owned entirely by CommandContext — no local WS here. */

  /* ----------------------------- Gamepad ---------------------------------- */
  // Proxy ref: gives useGamepad a WS-compatible object backed by CommandContext.
  // readyState uses a ref so the rAF loop always sees the current connection status.
  const _statusRef = React.useRef(status);
  useEffect(() => { _statusRef.current = status; }, [status]);
  const gamepadWsProxy = React.useRef<WebSocket | null>(null);
  if (!gamepadWsProxy.current) {
    gamepadWsProxy.current = {
      get readyState() {
        return _statusRef.current === 'connected' ? WebSocket.OPEN : WebSocket.CLOSED;
      },
      send: (data: string) => {
        try {
          const parsed = JSON.parse(data);
          sendCommand(parsed.command, parsed);
        } catch {}
      },
    } as unknown as WebSocket;
  }
  const gamepadState = useGamepad({ wsRef: gamepadWsProxy, paused: gamepadPaused || MOCK_WS });

  /* ---- Pi-side controller detection (polls /gamepad/status every 3 s) ---- */
  const apiBase = (process.env.NEXT_PUBLIC_API_URL ?? '').replace(/\/$/, '') || 'http://localhost:8000';
  const piGamepad = usePiGamepad(apiBase);

  /* ---------------- Autonomy API (HTTP wire → FastAPI :8000) --------------- */
  const autonomyApi = useMemo(() => {
    const apiBase = (process.env.NEXT_PUBLIC_API_URL ?? '').replace(/\/$/, '') || 'http://localhost:8000';
    return makeAutonomyApi(createHttpWire(apiBase));
  }, []);

  /* ---------------------- Send command helper with logs --------------------- */
  const sendCommandWithLog = useCallback(
    (command: string, angle: number = 0) => {
      if (MOCK_WS) {
        addCommand(`[MOCK] Sent: ${command}`);
        return;
      }
      // Delegate to CommandContext — it owns the WebSocket
      sendCommand(command, angle ? { angle } : undefined);
    },
    [addCommand, sendCommand]
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
  const isConnected = MOCK_WS || status === 'connected';

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

      <div className="relative">
        <Header
          batteryLevel={null}
          gamepadConnected={gamepadState.connected}
          gamepadName={gamepadState.name}
          gamepadPaused={gamepadPaused}
        />
        <div className="absolute top-1/2 right-4 -translate-y-1/2 z-10">
          <DemoModeToggle />
        </div>
      </div>

      <main className="cyber-content px-4 xl:px-8 py-4 overflow-x-hidden">
        <div className="max-w-[1600px] mx-auto space-y-6">

          {/* Connection Mode — three-way selector */}
          <ConnectionModeSelector />

          {/* Zone 1: Controls | Camera | Camera Controls */}
          <div className="grid grid-cols-1 xl:grid-cols-[auto_1fr_auto] gap-4 xl:gap-6 items-center">
            <div className="shrink-0 space-y-3">
              <CarControlPanel />
              <AutonomyPanel
                connected={isConnected}

                onStart={async (mode, params) => {
                  if (demoMode) {
                    addCommand(`[DEMO] autonomy-start (${mode})`);
                    // In demo mode, 'waypoints' mode triggers navigation via sim engine
                    if (mode === 'waypoints') {
                      const wps = ((params as any)?.waypoints ?? []) as { x: number; y: number }[];
                      simEngine.navigateTo(wps);
                    }
                    return;
                  }
                  try {
                    await autonomyApi.start(mode, params);
                    addCommand(`Sent: autonomy-start (${mode})`);
                    autonomyApi.maybeSuggestLights(Boolean((params as any)?.headlights));
                  } catch (e) {
                    addCommand(`autonomy-start failed: ${String(e)}`);
                  }
                }}
                onStop={async () => {
                  if (demoMode) {
                    simEngine.cancelNav();
                    addCommand('[DEMO] autonomy-stop');
                    return;
                  }
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
                piGamepad={piGamepad}
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

          {/* Zone 3: System Mode + Localization + Latency */}
          <div className="grid grid-cols-1 xl:grid-cols-3 gap-4 xl:gap-6">
            <ErrorBoundary>
              <SystemStatusPanel />
            </ErrorBoundary>
            <ErrorBoundary>
              <LocalizationPanel />
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
