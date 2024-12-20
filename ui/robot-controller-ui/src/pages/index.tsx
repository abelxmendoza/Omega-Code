/*
# File: src/pages/index.tsx
# Summary:
Main entry point for the robot controller application, providing UI for controlling the robot, viewing sensor data, and managing commands.
*/

import Head from 'next/head';
import React, { useState, useEffect, useRef } from 'react';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/SensorDashboard';
import CarControlPanel from '../components/CarControlPanel';
import CameraControlPanel from '../components/CameraControlPanel';
import { CommandLogProvider, useCommandLog } from '../components/CommandLogContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import LedModal from '../components/LedModal';
import { v4 as uuidv4 } from 'uuid';

const Home: React.FC = () => {
  const { addCommand } = useCommandLog();
  const [isLedModalOpen, setIsLedModalOpen] = useState(false);
  const ws = useRef<WebSocket | null>(null);

  // WebSocket connection logic
  const connectWebSocket = () => {
    ws.current = new WebSocket('wss://100.82.88.25:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
      const interval = setInterval(() => {
        if (ws.current?.readyState === WebSocket.OPEN) {
          ws.current.send(JSON.stringify({ type: 'keep-alive' }));
        }
      }, 25000);

      ws.current.onclose = () => {
        clearInterval(interval);
        console.log('WebSocket connection closed, reconnecting...');
        setTimeout(connectWebSocket, 1000);
      };

      ws.current.onerror = (error) => {
        console.error('WebSocket error:', error);
      };
    };

    ws.current.onmessage = (event) => {
      console.log('Received message:', event.data);
    };
  };

  useEffect(() => {
    connectWebSocket();
    return () => {
      ws.current?.close();
    };
  }, []);

  const sendCommand = (command: string, angle: number = 0) => {
    const requestId = uuidv4();
    if (ws.current?.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify({ command, angle, request_id: requestId }));
      addCommand(`${command} (ID: ${requestId})`);
    } else {
      console.error('WebSocket is not open');
    }
  };

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p':
          sendCommand(COMMAND.INCREASE_SPEED);
          break;
        case 'o':
          sendCommand(COMMAND.DECREASE_SPEED);
          break;
        case ' ':
          sendCommand(COMMAND.CMD_BUZZER);
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
  }, []);

  return (
    <CommandLogProvider>
      <div className="min-h-screen bg-gray-50">
        <Head>
          <title>Robot Controller</title>
          <meta name="description" content="Control your robot" />
          <link rel="icon" href="/favicon.ico" />
        </Head>

        <Header isConnected={true} batteryLevel={75} />
        <main className="p-4 space-y-4">
          {/* Top Control Panel */}
          <div className="flex justify-center items-center space-x-8">
            <div className="flex-shrink-0">
              <CarControlPanel sendCommand={sendCommand} />
            </div>
            <VideoFeed />
            <div className="flex-shrink-0">
              <CameraControlPanel sendCommand={sendCommand} />
            </div>
          </div>

          {/* SensorDashboard and SpeedControl - Adjusted Size */}
          <div className="flex justify-center items-center space-x-6 mt-6">
            <div className="w-1/3">
              <SensorDashboard />
            </div>
            <div className="w-1/4">
              <SpeedControl sendCommand={sendCommand} onOpenLedModal={() => setIsLedModalOpen(true)} />
            </div>
          </div>

          {/* CommandLog */}
          <div className="flex flex-col items-center space-y-4 mt-4">
            <CommandLog />
          </div>
        </main>
        {isLedModalOpen && (
          <LedModal isOpen={isLedModalOpen} onClose={() => setIsLedModalOpen(false)} />
        )}
      </div>
    </CommandLogProvider>
  );
};

export default Home;
