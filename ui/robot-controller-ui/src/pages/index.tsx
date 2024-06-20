import React, { useEffect } from 'react';
import Head from 'next/head';
import Header from '../components/Header';
import VideoFeed from '../components/VideoFeed';
import ControlPanel from '../components/ControlPanel';
import SpeedControl from '../components/SpeedControl';
import CommandLog from '../components/CommandLog';

const Home: React.FC = () => {
  const sendCommand = (command: string) => {
    fetch('http://localhost:8080/command', {  // Ensure the URL matches your Go server port
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ command }),
    }).then(response => {
      if (!response.ok) {
        console.error('Error sending command:', response.statusText);
      }
    }).catch(error => {
      console.error('Error sending command:', error);
    });
  };

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'w':
        case 'W':
          sendCommand('move-up');
          break;
        case 'a':
        case 'A':
          sendCommand('move-left');
          break;
        case 's':
        case 'S':
          sendCommand('move-down');
          break;
        case 'd':
        case 'D':
          sendCommand('move-right');
          break;
        case 'ArrowUp':
          sendCommand('camera-up');
          break;
        case 'ArrowLeft':
          sendCommand('camera-left');
          break;
        case 'ArrowDown':
          sendCommand('camera-down');
          break;
        case 'ArrowRight':
          sendCommand('camera-right');
          break;
        case 'p':
        case 'P':
          sendCommand('increase-speed');
          break;
        case 'o':
        case 'O':
          sendCommand('decrease-speed');
          break;
        case ' ':
          sendCommand('stop');
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
  const handleCameraControl = (command: string) => () => sendCommand(command);

  return (
    <div className="min-h-screen bg-gray-50">
      <Head>
        <title>Robot Controller</title>
        <meta name="description" content="Control your robot" />
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <Header />
      <main className="p-4 space-y-4">
        <div className="flex justify-between items-center space-x-8">
          <div className="flex-shrink-0">
            <ControlPanel
              onUp={handleCarControl('move-up')}
              onDown={handleCarControl('move-down')}
              onLeft={handleCarControl('move-left')}
              onRight={handleCarControl('move-right')}
            />
          </div>
          <VideoFeed />
          <div className="flex-shrink-0">
            <ControlPanel
              onUp={handleCameraControl('camera-up')}
              onDown={handleCameraControl('camera-down')}
              onLeft={handleCameraControl('camera-left')}
              onRight={handleCameraControl('camera-right')}
            />
          </div>
        </div>
        <div className="flex flex-col items-center space-y-4 mt-4">
          <SpeedControl />
          <CommandLog />
        </div>
      </main>
    </div>
  );
};

export default Home;

