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

/**
 * Home Component
 * 
 * This component is the main entry point for the robot controller application.
 * It provides the UI for controlling the robot, viewing sensor data, and managing command logs.
 * The component handles sending commands to the robot and integrates multiple sub-components.
 */
const Home: React.FC = () => {
  const { addCommand } = useCommandLog();
  const [isLedModalOpen, setIsLedModalOpen] = useState(false); // State to manage LED modal visibility
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Initialize WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      console.log('Received message:', event.data);
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  /**
   * Sends a command to the robot.
   * @param {string} command - The command to send.
   * @param {number} [angle=0] - Optional angle parameter for certain commands.
   */
  const sendCommand = (command: string, angle: number = 0) => {
    const requestId = uuidv4();
    const message = JSON.stringify({ command, angle, request_id: requestId });

    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(message);
      console.log(`Command sent: ${command}`);
      addCommand(`${command} (ID: ${requestId})`);
    } else {
      console.error('WebSocket is not open');
    }
  };

  /**
   * Handles key down events to trigger commands.
   * @param {KeyboardEvent} event - The keyboard event.
   */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          sendCommand(COMMAND.INCREASE_SPEED);
          break;
        case 'o':
        case 'O':
          sendCommand(COMMAND.DECREASE_SPEED);
          break;
        case ' ':
          sendCommand(COMMAND.CMD_BUZZER);
          break;
        case 'i':
        case 'I':
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
          <div className="flex justify-center items-center space-x-8">
            <div className="flex-shrink-0">
              <CarControlPanel sendCommand={sendCommand} />
            </div>
            <VideoFeed />
            <div className="flex-shrink-0">
              <CameraControlPanel sendCommand={sendCommand} />
            </div>
          </div>
          <div className="flex justify-center items-center space-x-8 mt-4">
            <SensorDashboard />
            <SpeedControl sendCommand={sendCommand} onOpenLedModal={() => setIsLedModalOpen(true)} />
          </div>
          <div className="flex flex-col items-center space-y-4 mt-4">
            <CommandLog />
          </div>
        </main>
        {isLedModalOpen && <LedModal isOpen={isLedModalOpen} sendCommand={sendCommand} onClose={() => setIsLedModalOpen(false)} />}
      </div>
    </CommandLogProvider>
  );
};

export default Home;
