// src/pages/index.tsx

/*
This is the main application page that sets up the robot control interface.
It includes the video feed, control panels, speed control, and command log.
*/

import React, { useEffect } from 'react';
import Head from 'next/head';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';
import { CommandLogProvider, useCommandLog } from '../components/CommandLogContext';
import { COMMAND } from '../control_definitions'; // Import command definitions

const Home: React.FC = () => {
  const { addCommand } = useCommandLog(); // Use context to add command to log

  // Function to send commands to the server
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
        addCommand(command); // Add command to log on successful send
      }
    }).catch(error => {
      console.error('Error sending command:', error);
    });
  };

  // Effect hook to handle key press events
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'w':
        case 'W':
          sendCommand(COMMAND.MOVE_UP);
          break;
        case 'a':
        case 'A':
          sendCommand(COMMAND.MOVE_LEFT);
          break;
        case 's':
        case 'S':
          sendCommand(COMMAND.MOVE_DOWN);
          break;
        case 'd':
        case 'D':
          sendCommand(COMMAND.MOVE_RIGHT);
          break;
        case 'ArrowUp':
          sendCommand(COMMAND.CMD_SERVO_VERTICAL, 10); // Adjust angle as needed
          break;
        case 'ArrowLeft':
          sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, 10); // Adjust angle as needed
          break;
        case 'ArrowDown':
          sendCommand(COMMAND.CMD_SERVO_VERTICAL, -10); // Adjust angle as needed
          break;
        case 'ArrowRight':
          sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, -10); // Adjust angle as needed
          break;
        case 'p':
        case 'P':
          sendCommand(COMMAND.INCREASE_SPEED);
          break;
        case 'o':
        case 'O':
          sendCommand(COMMAND.DECREASE_SPEED);
          break;
        case ' ':
          sendCommand(COMMAND.HONK);
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

  const handleCarControl = (command: string) => () => sendCommand(command);
  const handleCameraControl = (command: string, angle: number) => () => sendCommand(command, angle);

  return (
    <CommandLogProvider> {/* Wrap components with CommandLogProvider */}
      <div className="min-h-screen bg-gray-50">
        <Head>
          <title>Robot Controller</title>
          <meta name="description" content="Control your robot" />
          <link rel="icon" href="/favicon.ico" />
        </Head>

        <Header />
        <main className="p-4 space-y-4">
          <div className="flex justify-center items-center space-x-8">
            <div className="flex-shrink-0">
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
            <div className="flex-shrink-0">
              <ControlPanel
                onUp={handleCameraControl(COMMAND.CMD_SERVO_VERTICAL, 10)}
                onDown={handleCameraControl(COMMAND.CMD_SERVO_VERTICAL, -10)}
                onLeft={handleCameraControl(COMMAND.CMD_SERVO_HORIZONTAL, 10)}
                onRight={handleCameraControl(COMMAND.CMD_SERVO_HORIZONTAL, -10)}
                labels={{ up: '↑', down: '↓', left: '←', right: '→' }}
                controlType="arrows"
              />
            </div>
          </div>
          <div className="flex flex-col items-center space-y-4 mt-4">
            <SpeedControl sendCommand={sendCommand} />
            <CommandLog />
          </div>
        </main>
      </div>
    </CommandLogProvider>
  );
};

export default Home;
