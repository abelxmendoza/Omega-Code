// src/components/SpeedControl.tsx

/*
This component provides a control interface for adjusting the speed of the robot and honking the horn.
It handles both keyboard and button inputs for increasing, decreasing speed, and honking.
*/

import React, { useState, useEffect } from 'react';
import { COMMAND } from '../control_definitions'; // Import command definitions

const SpeedControl: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  const [speed, setSpeed] = useState(0);
  const [accelerating, setAccelerating] = useState(false);
  const [decelerating, setDecelerating] = useState(false);
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    let accelerateInterval: NodeJS.Timeout;
    let decelerateInterval: NodeJS.Timeout;

    // Handle key down events for controlling speed and honking
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          if (!accelerating) {
            setAccelerating(true);
            setActiveKey('p');
            accelerateInterval = setInterval(() => {
              setSpeed(prevSpeed => {
                const newSpeed = Math.min(prevSpeed + 1, 100);
                sendCommand(`${COMMAND.INCREASE_SPEED}-${newSpeed}`);
                return newSpeed;
              });
            }, 100); // Adjust this value to control the acceleration speed
          }
          break;
        case 'o':
        case 'O':
          if (!decelerating) {
            setDecelerating(true);
            setActiveKey('o');
            decelerateInterval = setInterval(() => {
              setSpeed(prevSpeed => {
                const newSpeed = Math.max(prevSpeed - 1, 0);
                sendCommand(`${COMMAND.DECREASE_SPEED}-${newSpeed}`);
                return newSpeed;
              });
            }, 100); // Adjust this value to control the deceleration speed
          }
          break;
        case ' ':
          setActiveKey(' ');
          sendCommand(COMMAND.HONK);
          break;
        default:
          break;
      }
    };

    // Handle key up events to stop accelerating or decelerating
    const handleKeyUp = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          setAccelerating(false);
          setActiveKey(null);
          clearInterval(accelerateInterval);
          break;
        case 'o':
        case 'O':
          setDecelerating(false);
          setActiveKey(null);
          clearInterval(decelerateInterval);
          break;
        case ' ':
          setActiveKey(null);
          break;
        default:
          break;
      }
    };

    // Add event listeners for key down and key up events
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    // Cleanup event listeners on component unmount
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      clearInterval(accelerateInterval);
      clearInterval(decelerateInterval);
    };
  }, [accelerating, decelerating, sendCommand]);

  // Get the progress bar color based on the current speed
  const getProgressColor = () => {
    if (speed <= 20) return 'bg-red-500';
    if (speed <= 60) return 'bg-yellow-500';
    return 'bg-green-500';
  };

  // Get the button class based on the active key
  const getButtonClass = (key: string) => {
    return activeKey === key ? 'bg-red-500' : 'bg-blue-500';
  };

  return (
    <div className="flex flex-col items-center space-y-4">
      <div className="flex items-center space-x-2 w-full">
        <span>Speed:</span>
        <div className="w-full bg-gray-300 rounded h-4 flex items-center relative">
          <div
            className={`h-full rounded ${getProgressColor()} transition-all duration-500 ease-in-out`}
            style={{ width: `${speed}%` }}
          ></div>
          <div className="absolute inset-0 flex justify-center items-center text-white font-bold">
            {speed}%
          </div>
        </div>
      </div>
      <div className="flex flex-col items-center">
        <div className="flex space-x-4">
          <button
            className={`w-16 h-16 rounded-lg ${getButtonClass('o')} text-white flex flex-col items-center justify-center`}
            onClick={() => sendCommand(COMMAND.DECREASE_SPEED)}
          >
            <span>O</span>
            <span>(brake)</span>
          </button>
          <button
            className={`w-16 h-16 rounded-lg ${getButtonClass('p')} text-white flex flex-col items-center justify-center`}
            onClick={() => sendCommand(COMMAND.INCREASE_SPEED)}
          >
            <span>P</span>
            <span>(gas)</span>
          </button>
        </div>
        <button
          className={`w-32 h-12 rounded-lg ${getButtonClass(' ')} text-white mt-4 flex items-center justify-center`}
          onClick={() => sendCommand(COMMAND.HONK)}
        >
          Space (Honk)
        </button>
      </div>
    </div>
  );
};

export default SpeedControl;
