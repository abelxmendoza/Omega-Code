import React, { useState, useEffect } from 'react';

const Accelerometer: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  const [speed, setSpeed] = useState(0);
  const [accelerating, setAccelerating] = useState(false);
  const [decelerating, setDecelerating] = useState(false);

  useEffect(() => {
    let accelerateInterval: NodeJS.Timeout;
    let decelerateInterval: NodeJS.Timeout;

    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          if (!accelerating) {
            setAccelerating(true);
            accelerateInterval = setInterval(() => {
              setSpeed(prevSpeed => {
                const newSpeed = Math.min(prevSpeed + 1, 100);
                sendCommand(`set-speed-${newSpeed}`);
                return newSpeed;
              });
            }, 100); // Adjust this value to control the acceleration speed
          }
          break;
        case 'o':
        case 'O':
          if (!decelerating) {
            setDecelerating(true);
            decelerateInterval = setInterval(() => {
              setSpeed(prevSpeed => {
                const newSpeed = Math.max(prevSpeed - 1, 0);
                sendCommand(`set-speed-${newSpeed}`);
                return newSpeed;
              });
            }, 100); // Adjust this value to control the deceleration speed
          }
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
          clearInterval(accelerateInterval);
          break;
        case 'o':
        case 'O':
          setDecelerating(false);
          clearInterval(decelerateInterval);
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
    if (speed <= 20) return 'bg-red-500';
    if (speed <= 60) return 'bg-yellow-500';
    return 'bg-green-500';
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
            className="w-16 h-16 rounded-lg bg-blue-500 text-white flex items-center justify-center"
            onClick={() => sendCommand('accelerate')}
          >
            P
          </button>
          <button
            className="w-16 h-16 rounded-lg bg-blue-500 text-white flex items-center justify-center"
            onClick={() => sendCommand('decelerate')}
          >
            O
          </button>
        </div>
        <button
          className="w-32 h-12 rounded-lg bg-blue-500 text-white mt-4 flex items-center justify-center"
          onClick={() => sendCommand('honk')}
        >
          Space (Honk)
        </button>
      </div>
    </div>
  );
};

export default Accelerometer;
