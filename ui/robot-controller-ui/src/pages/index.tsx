/*
# File: /src/pages/index.tsx
# Summary:
Main entry for the robot controller UI.
- Client-only CameraFrame (avoids SSR crashes)
- Robot controls, logs, sensors, lighting modal
- WebSocket keep-alive + hotkeys
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

// ⬇️ Load CameraFrame only in the browser (prevents SSR issues)
const CameraFrame = dynamic(() => import('../components/CameraFrame'), {
  ssr: false,
  loading: () => (
    <div className="w-[720px] max-w-[42vw]">
      <div className="relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden" style={{ paddingTop: '56.25%' }}>
        <div className="absolute inset-0 flex items-center justify-center text-white/70 text-sm">
          Loading camera…
        </div>
      </div>
    </div>
  ),
});

/** Pick endpoint by profile: lan | tailscale | local */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local';
  return (
    process.env[`${base}_${profile.toUpperCase()}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
};

// Video stream URL (used by CameraFrame)
const videoUrl = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL');

export default function Home() {
  const { addCommand } = useCommand();
  const [isLedModalOpen, setIsLedModalOpen] = useState(false);
  const ws = useRef<WebSocket | null>(null);

  // --- WebSocket to your backend command endpoint ---
  const connectWebSocket = useCallback(() => {
    ws.current = new WebSocket('wss://100.82.88.25:8080/ws');

    ws.current.onopen = () => {
      addCommand('WebSocket connection established');

      const interval = setInterval(() => {
        if (ws.current?.readyState === WebSocket.OPEN) {
          ws.current.send(JSON.stringify({ type: 'keep-alive' }));
        }
      }, 25_000);

      if (ws.current) {
        ws.current.onclose = () => {
          clearInterval(interval);
          addCommand('WebSocket connection closed. Reconnecting...');
          setTimeout(connectWebSocket, 1000);
        };

        ws.current.onerror = (error) => {
          addCommand(`WebSocket error: ${String(error)}`);
        };

        ws.current.onmessage = (event) => {
          try {
            const message = JSON.parse(event.data);
            if (message.command) addCommand(`Received: ${message.command}`);
          } catch {
            addCommand('Error parsing WebSocket message');
          }
        };
      }
    };
  }, [addCommand]);

  useEffect(() => {
    connectWebSocket();
    return () => ws.current?.close();
  }, [connectWebSocket]);

  // --- Send command helper with logging ---
  const sendCommandWithLog = useCallback(
    (command: string, angle: number = 0) => {
      const requestId = uuidv4();
      if (ws.current?.readyState === WebSocket.OPEN) {
        ws.current.send(JSON.stringify({ command, angle, request_id: requestId }));
        addCommand(`Sent: ${command} (ID: ${requestId})`);
      } else {
        addCommand('Failed to send command: WebSocket is not open');
      }
    },
    [addCommand]
  );

  // --- Hotkeys ---
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
              src={videoUrl}
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
