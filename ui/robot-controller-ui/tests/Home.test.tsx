/**
 * File Path: src/pages/index.tsx
 * 
 * Home Component
 * 
 * This component is the main entry point for the robot controller application.
 * It provides the UI for controlling the robot, viewing sensor data, and managing command logs.
 * The component handles sending commands to the robot and integrates multiple sub-components.
 */

import Head from 'next/head';
import React, { useState, useEffect } from 'react';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/SensorDashboard';
import CarControlPanel from '../components/CarControlPanel'; // Import CarControlPanel
import CameraControlPanel from '../components/CameraControlPanel'; // Import CameraControlPanel
import { CommandLogProvider, useCommandLog } from '../components/CommandLogContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import LedModal from '../components/LedModal'; // Import LedModal
import { v4 as uuidv4 } from 'uuid';

const Home: React.FC = () => {
  const { addCommand } = useCommandLog();
  const [isLedModalOpen, setIsLedModalOpen] = useState(false); // State to manage LED modal visibility

  /**
   * Sends a command to the robot.
   * @param {string} command - The command to send.
   * @param {number} [angle=0] - Optional angle parameter for certain commands.
   */
  const sendCommand = (command: string, angle: number = 0) => {
    const requestId = uuidv4();
    fetch('https://localhost:8080/command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ command, angle, request_id: requestId }),
    })
      .then((response) => {
        if (!response.ok) {
          console.error('Error sending command:', response.statusText);
        } else {
          console.log(`Command sent: ${command}`);
          addCommand(`${command} (ID: ${requestId})`);
        }
      })
      .catch((error) => {
        console.error('Error sending command:', error);
      });
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
            <div data-testid="video-feed">
              <VideoFeed />
            </div>
            <div className="flex-shrink-0">
              <CameraControlPanel sendCommand={sendCommand} />
            </div>
          </div>
          <div className="flex justify-center items-center space-x-8 mt-4">
            <SensorDashboard />
            <div data-testid="speed-control">
              <SpeedControl sendCommand={sendCommand} onOpenLedModal={() => setIsLedModalOpen(true)} />
            </div>
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
