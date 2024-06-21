// File: /Omega-Code/ui/robot-controller-ui/components/SpeedControl.tsx
import React, { useState, useEffect } from 'react';

const SpeedControl: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  const [speed, setSpeed] = useState(0);
  const [accelerating, setAccelerating] = useState(false);
  const [decelerating, setDecelerating] = useState(false);
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    let accelerateInterval: NodeJS.Timeout;
    let decelerateInterval: NodeJS.Timeout;

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
                sendCommand(`set-speed-${newSpeed}`);
                return newSpeed;
              });
            }, 100); 
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
                sendCommand(`set-speed-${newSpeed}`);
                return newSpeed;
              });
            }, 100); 
          }
          break;
        case ' ':
          setActiveKey(' ');
          sendCommand('honk');
          break;
        default:
          break;
      }
    };

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

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      clearInterval(accelerateInterval);
      clearInterval(decelerateInterval);
    };
  }, [accelerating, decelerating, sendCommand]);

  const getProgressColor = () => {
    if (accelerating) return 'bg-green-500';
    if (decelerating) return 'bg-red-500';
    return 'bg-gray-300';
  };

  const getButtonClass = (key: string) => {
    return activeKey === key ? 'bg-red-500' : 'bg-blue-500';
  };

  return (
    <div className="flex flex-col items-center space-y-4">
      <div className="flex items-center space-x-2 w-full">
        <span>Speed:</span>
        <div className="w-full bg-gray-300 rounded h-4 flex items-center">
          <div
            className={`h-full rounded ${getProgressColor()}`}
            style={{ width: `${speed}%` }}
          ></div>
        </div>
        <span>{speed}</span>
      </div>
      <div className="flex flex-col items-center">
        <div className="flex space-x-4">
          <button
            className={`w-16 h-16 rounded-lg ${getButtonClass('o')} text-white flex flex-col items-center justify-center`}
            onClick={() => sendCommand('decelerate')}
          >
            <span>O</span>
            <span>(brake)</span>
          </button>
          <button
            className={`w-16 h-16 rounded-lg ${getButtonClass('p')} text-white flex flex-col items-center justify-center`}
            onClick={() => sendCommand('accelerate')}
          >
            <span>P</span>
            <span>(gas)</span>
          </button>
        </div>
        <button
          className={`w-32 h-12 rounded-lg ${getButtonClass(' ')} text-white mt-4 flex items-center justify-center`}
          onClick={() => sendCommand('honk')}
        >
          Space (Honk)
        </button>
      </div>
    </div>
  );
};

export default SpeedControl;
