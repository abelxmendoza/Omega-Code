import React, { useEffect, useState } from 'react';
import Head from 'next/head';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';
import LedModal from '../components/LedModal';
import { useCommandLog } from '../components/CommandLogContext';

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
          break;
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

  return (
    <div className="min-h-screen bg-gray-50">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <Header isConnected={isConnected} batteryLevel={batteryLife} />

      <main className="p-4 space-y-4">
        <div className="flex justify-center items-center space-x-8">
          <div className="flex-shrink-0 mr-20">
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
          <div className="flex-shrink-0 ml-20">
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
          <div className="flex items-center space-x-4">
            <button
              className="w-16 h-16 rounded-lg bg-blue-500 text-white flex flex-col items-center justify-center"
              onClick={() => setIsModalOpen(true)}
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
