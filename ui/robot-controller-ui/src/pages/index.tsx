/*
# File: /src/pages/index.tsx
# Summary:
Main entry point for the robot controller application.
Provides a UI for robot control, real-time command logging, and sensor data visualization.
Manages WebSocket communication for command sending and response handling.
*/

import Head from 'next/head';
import React, { useState, useEffect, useRef, useCallback } from 'react';
import SpeedControl from '../components/control/SpeedControl';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/sensors/SensorDashboard';
import CarControlPanel from '../components/control/CarControlPanel';
import CameraControlPanel from '../components/control/CameraControlPanel';
import { useCommand } from '../context/CommandContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
// import VideoFeed from '../components/VideoFeed'; // replaced by CameraFrame on this page
import CameraFrame from '../components/CameraFrame';
import LedModal from '../components/lighting/LedModal';
import { v4 as uuidv4 } from 'uuid';

// Utility to pick endpoints by profile (lan | tailscale | local)
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

const Home: React.FC = () => {
  const { sendCommand, addCommand } = useCommand();
  const [isLedModalOpen, setIsLedModalOpen] = useState(false);
  const ws = useRef<WebSocket | null>(null);

  const connectWebSocket = useCallback(() => {
    ws.current = new WebSocket('wss://100.82.88.25:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
      addCommand('WebSocket connection established');

      const interval = setInterval(() => {
        if (ws.current?.readyState === WebSocket.OPEN) {
          ws.current.send(JSON.stringify({ type: 'keep-alive' }));
        }
      }, 25000);

      if (ws.current) {
        ws.current.onclose = () => {
          clearInterval(interval);
          console.warn('WebSocket connection closed. Reconnecting...');
          addCommand('WebSocket connection closed. Reconnecting...');
          setTimeout(connectWebSocket, 1000);
        };

        ws.current.onerror = (error) => {
          console.error('WebSocket error:', error);
          addCommand(`WebSocket error: ${error}`);
        };

        ws.current.onmessage = (event) => {
          try {
            const message = JSON.parse(event.data);
            if (message.command) {
              addCommand(`Received: ${message.command}`);
            }
          } catch (error) {
            console.error('Error parsing WebSocket message:', error);
            addCommand('Error parsing WebSocket message');
          }
        };
      }
    };
  }, [addCommand]);

  useEffect(() => {
    connectWebSocket();
    return () => {
      ws.current?.close();
    };
  }, [connectWebSocket]);

  const sendCommandWithLog = useCallback(
    (command: string, angle: number = 0) => {
      const requestId = uuidv4();
      if (ws.current?.readyState === WebSocket.OPEN) {
        const payload = { command, angle, request_id: requestId };
        ws.current.send(JSON.stringify(payload));
        addCommand(`Sent: ${command} (ID: ${requestId})`);
      } else {
        console.error('WebSocket is not open');
        addCommand('Failed to send command: WebSocket is not open');
      }
    },
    [addCommand]
  );

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p':
          sendCommandWithLog(COMMAND.INCREASE_SPEED);
          break;
        case 'o':
          sendCommandWithLog(COMMAND.DECREASE_SPEED);
          break;
        case ' ':
          sendCommandWithLog(COMMAND.CMD_BUZZER);
          break;
        case 'i':
          setIsLedModalOpen(true);
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [sendCommandWithLog]);

  return (
    <div className="min-h-screen bg-gray-50">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <Header isConnected={true} batteryLevel={75} />

      <main className="p-4 space-y-4">
        <div className="flex justify-center items-center space-x-8">
          <div className="flex-shrink-0">
            <CarControlPanel sendCommand={sendCommandWithLog} />
          </div>

          {/* Replaced <VideoFeed /> with framed camera */}
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
};

export default Home;
