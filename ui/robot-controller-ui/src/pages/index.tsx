/*
# File: /src/pages/index.tsx
# Summary:
Main entry point for the robot controller application.
Provides a UI for robot control, real-time command logging, and sensor data visualization.
Manages WebSocket communication for command sending and response handling.
*/

import Head from 'next/head';
import React, { useState, useEffect, useRef } from 'react';
import SpeedControl from '../components/control/SpeedControl';
import CommandLog from '../components/CommandLog';
import SensorDashboard from '../components/sensors/SensorDashboard';
import CarControlPanel from '../components/control/CarControlPanel';
import CameraControlPanel from '../components/control/CameraControlPanel';
import { useCommand } from '../context/CommandContext';
import { COMMAND } from '../control_definitions';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import LedModal from '../components/lighting/LedModal';
import { v4 as uuidv4 } from 'uuid';

const Home: React.FC = () => {
  // Access methods from CommandContext for sending commands and logging
  const { sendCommand, addCommand } = useCommand();

  // State for managing the visibility of the LED configuration modal
  const [isLedModalOpen, setIsLedModalOpen] = useState(false);

  // WebSocket instance reference for direct communication
  const ws = useRef<WebSocket | null>(null);

  /*
  # Function: connectWebSocket
  Establishes a WebSocket connection and manages lifecycle events such as connection, message handling, errors, and reconnection.
  */
  const connectWebSocket = () => {
    ws.current = new WebSocket('wss://100.82.88.25:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
      addCommand('WebSocket connection established');

      // Keep the WebSocket connection alive with periodic pings
      const interval = setInterval(() => {
        if (ws.current?.readyState === WebSocket.OPEN) {
          ws.current.send(JSON.stringify({ type: 'keep-alive' }));
        }
      }, 25000); // 25-second ping interval

      // Handle WebSocket closure and attempt reconnection
      ws.current.onclose = () => {
        clearInterval(interval);
        console.warn('WebSocket connection closed. Reconnecting...');
        addCommand('WebSocket connection closed. Reconnecting...');
        setTimeout(connectWebSocket, 1000); // Reconnect after 1 second
      };

      // Log WebSocket errors
      ws.current.onerror = (error) => {
        console.error('WebSocket error:', error);
        addCommand(`WebSocket error: ${error}`);
      };
    };

    // Handle incoming WebSocket messages
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
  };

  // Initialize WebSocket connection when the component mounts
  useEffect(() => {
    connectWebSocket();
    return () => {
      ws.current?.close(); // Clean up WebSocket connection on unmount
    };
  }, []);

  /*
  # Function: sendCommandWithLog
  Sends a command to the WebSocket server, adds a unique ID for tracking, and logs the command.
  - command: The command string to send.
  - angle: Optional angle value for commands requiring it.
  */
  const sendCommandWithLog = (command: string, angle: number = 0) => {
    const requestId = uuidv4(); // Generate a unique request ID
    if (ws.current?.readyState === WebSocket.OPEN) {
      const payload = { command, angle, request_id: requestId };
      ws.current.send(JSON.stringify(payload));
      addCommand(`Sent: ${command} (ID: ${requestId})`);
    } else {
      console.error('WebSocket is not open');
      addCommand('Failed to send command: WebSocket is not open');
    }
  };

  /*
  # Effect: Keyboard Shortcuts
  Adds keyboard event listeners for sending predefined commands.
  */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p': // Increase speed
          sendCommandWithLog(COMMAND.INCREASE_SPEED);
          break;
        case 'o': // Decrease speed
          sendCommandWithLog(COMMAND.DECREASE_SPEED);
          break;
        case ' ': // Activate buzzer
          sendCommandWithLog(COMMAND.CMD_BUZZER);
          break;
        case 'i': // Open LED modal
          setIsLedModalOpen(true);
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown); // Clean up on component unmount
    };
  }, []);

  return (
    <div className="min-h-screen bg-gray-50">
      {/* Meta Information */}
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      {/* Header: Display connection status and battery level */}
      <Header isConnected={true} batteryLevel={75} />

      <main className="p-4 space-y-4">
        {/* Top Control Panel */}
        <div className="flex justify-center items-center space-x-8">
          <div className="flex-shrink-0">
            <CarControlPanel sendCommand={sendCommandWithLog} />
          </div>
          <VideoFeed />
          <div className="flex-shrink-0">
            <CameraControlPanel sendCommand={sendCommandWithLog} />
          </div>
        </div>

        {/* Sensor Dashboard and Speed Control */}
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

        {/* Command Log */}
        <div className="flex flex-col items-center space-y-4 mt-4">
          <CommandLog />
        </div>
      </main>

      {/* LED Configuration Modal */}
      {isLedModalOpen && (
        <LedModal isOpen={isLedModalOpen} onClose={() => setIsLedModalOpen(false)} />
      )}
    </div>
  );
};

export default Home;
