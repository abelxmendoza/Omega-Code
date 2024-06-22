import React, { useEffect, useState } from 'react';
import Head from 'next/head';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';
import LedModal from '../components/LedModal';
import { useCommandLog } from '../components/CommandLogContext';
import { COMMAND } from '@/control_definitions'; // Use alias defined in tsconfig.json


const Home: React.FC = () => {
  const { addCommand } = useCommandLog();
  const [isConnected, setIsConnected] = useState(true);
  const [batteryLife, setBatteryLife] = useState(80);
  const [isModalOpen, setIsModalOpen] = useState(false);

  const sendCommand = (command: string, angle: number = 0) => {
    fetch('https://localhost:8080/command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ command, angle }),
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
      let angle = 0;

      switch (event.key) {
        case 'w':
        case 'W':
          command = COMMAND.MOVE_UP;
          break;
        case 'a':
        case 'A':
          command = COMMAND.MOVE_LEFT;
          break;
        case 's':
        case 'S':
          command = COMMAND.MOVE_DOWN;
          break;
        case 'd':
        case 'D':
          command = COMMAND.MOVE_RIGHT;
          break;
        case 'ArrowUp':
          command = COMMAND.CMD_SERVO_VERTICAL;
          angle = 10; // Adjust angle as needed
          break;
        case 'ArrowLeft':
          command = COMMAND.CMD_SERVO_HORIZONTAL;
          angle = -10; // Adjust angle as needed
          break;
        case 'ArrowDown':
          command = COMMAND.CMD_SERVO_VERTICAL;
          angle = -10; // Adjust angle as needed
          break;
        case 'ArrowRight':
          command = COMMAND.CMD_SERVO_HORIZONTAL;
          angle = 10; // Adjust angle as needed
          break;
        case 'p':
        case 'P':
          command = COMMAND.INCREASE_SPEED;
          break;
        case 'o':
        case 'O':
          command = COMMAND.DECREASE_SPEED;
          break;
        case ' ':
          command = COMMAND.HONK;
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
        sendCommand(command, angle);
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
              onUp={handleCarControl(COMMAND.MOVE_UP)}
              onDown={handleCarControl(COMMAND.MOVE_DOWN)}
              onLeft={handleCarControl(COMMAND.MOVE_LEFT)}
              onRight={handleCarControl(COMMAND.MOVE_RIGHT)}
              labels={{ up: 'W', down: 'S', left: 'A', right: 'D' }}
              controlType="wasd"
            />
          </div>
          <VideoFeed />
          <div className="flex-shrink-0 ml-20">
            <ControlPanel
              onUp={handleCameraControl(COMMAND.CMD_SERVO_VERTICAL)}
              onDown={handleCameraControl(COMMAND.CMD_SERVO_VERTICAL)}
              onLeft={handleCameraControl(COMMAND.CMD_SERVO_HORIZONTAL)}
              onRight={handleCameraControl(COMMAND.CMD_SERVO_HORIZONTAL)}
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
