// File: /Omega-Code/ui/robot-controller-ui/src/pages/index.tsx
import React, { useEffect, useState } from 'react';
import Head from 'next/head';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';
import { useCommandLog } from '../components/CommandLogContext';
import LedModal from '../components/LedModal';

const Home: React.FC = () => {
  const { addCommand } = useCommandLog();
  const [isConnected, setIsConnected] = useState(true);
  const [batteryLife, setBatteryLife] = useState(80);
  const [isModalOpen, setIsModalOpen] = useState(false);

  const sendCommand = (command: string) => {
    fetch('https://localhost:8080/command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ command }),
    }).then(response => {
      if (!response.ok) {
        console.error('Error sending command:', response.statusText);
      } else {
        console.log(`Command sent: ${command}`);
        addCommand(command);
      }
    }).catch(error => {
      console.error('Error sending command:', error);
      setIsConnected(false);
    });
  };

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      let command = '';
      switch (event.key) {
        case 'w':
        case 'W':
          command = 'move-up';
          break;
        case 'a':
        case 'A':
          command = 'move-left';
          break;
        case 's':
        case 'S':
          command = 'move-down';
          break;
        case 'd':
        case 'D':
          command = 'move-right';
          break;
        case 'ArrowUp':
          command = 'camera-up';
          break;
        case 'ArrowLeft':
          command = 'camera-left';
          break;
        case 'ArrowDown':
          command = 'camera-down';
          break;
        case 'ArrowRight':
          command = 'camera-right';
          break;
        case 'p':
        case 'P':
          command = 'increase-speed';
          break;
        case 'o':
        case 'O':
          command = 'decrease-speed';
          break;
        case ' ':
          command = 'honk';
          break;
        case 'i':
        case 'I':
          setIsModalOpen(true);
          return; // Exit the function early, as we don't want to send a command for 'i'
        default:
          break;
      }

      if (command) {
        console.log(`Sending command: ${command}`);
        sendCommand(command);
      }
    };

    window.addEventListener('keydown', handleKeyDown);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, []);

  const handleCarControl = (command: string) => () => sendCommand(command);
  const handleCameraControl = (command: string) => () => sendCommand(command);

  const batteryClass = batteryLife > 20 ? 'bg-blue-500' : 'bg-red-500';
  const batteryStyle = batteryLife > 20 ? 'neon-blue' : 'black';
  const batteryBarClass = `h-4 rounded ${batteryClass}`;

  return (
    <div className="min-h-screen bg-gray-50">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <Header isConnected={isConnected} batteryLevel={batteryLife} />
      <main className="p-4 space-y-4">
        <div className="flex justify-between items-center space-x-8">
          <div className="flex-shrink-0">
            <ControlPanel
              onUp={handleCarControl('move-up')}
              onDown={handleCarControl('move-down')}
              onLeft={handleCarControl('move-left')}
              onRight={handleCarControl('move-right')}
              labels={{ up: 'W', down: 'S', left: 'A', right: 'D' }}
              controlType="wasd"
            />
          </div>
          <VideoFeed />
          <div className="flex-shrink-0">
            <ControlPanel
              onUp={handleCameraControl('camera-up')}
              onDown={handleCameraControl('camera-down')}
              onLeft={handleCameraControl('camera-left')}
              onRight={handleCameraControl('camera-right')}
              labels={{ up: '↑', down: '↓', left: '←', right: '→' }}
              controlType="arrows"
            />
          </div>
        </div>
        <div className="flex flex-col items-center space-y-4 mt-4">
          <div className="flex space-x-4 items-center">
            <button
              onClick={() => setIsModalOpen(true)}
              className="w-16 h-16 rounded-lg bg-blue-500 text-white flex flex-col items-center justify-center"
            >
              <span>I</span>
              <span>(LED)</span>
            </button>
            <SpeedControl sendCommand={sendCommand} />
          </div>
          <CommandLog />
        </div>
      </main>
      <LedModal
        sendCommand={sendCommand}
        isOpen={isModalOpen}
        onClose={() => setIsModalOpen(false)}
      />
    </div>
  );
};

export default Home;
