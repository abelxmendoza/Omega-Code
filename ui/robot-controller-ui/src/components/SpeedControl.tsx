import React, { useState, useEffect } from 'react';
import { COMMAND } from '../control_definitions';
import { debounce } from 'lodash';

/**
 * SpeedControl Component
 * 
 * This component provides controls for adjusting the speed of the robot.
 */
const SpeedControl: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  const [speed, setSpeed] = useState(0);
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    let accelerateInterval: NodeJS.Timeout | undefined;
    let decelerateInterval: NodeJS.Timeout | undefined;

    const handleKeyDown = debounce((event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          if (!accelerateInterval) {
            setActiveKey('p');
            accelerateInterval = setInterval(() => {
              setSpeed((prevSpeed) => {
                const newSpeed = Math.min(prevSpeed + 5, 100); // Accelerate faster
                sendCommand(`${COMMAND.INCREASE_SPEED}-${newSpeed}`);
                return newSpeed;
              });
            }, 100); // Faster interval for acceleration
          }
          break;
        case 'o':
        case 'O':
          if (!decelerateInterval) {
            setActiveKey('o');
            decelerateInterval = setInterval(() => {
              setSpeed((prevSpeed) => {
                const newSpeed = Math.max(prevSpeed - 10, 0); // Decelerate faster
                sendCommand(`${COMMAND.DECREASE_SPEED}-${newSpeed}`);
                return newSpeed;
              });
            }, 100); // Faster interval for deceleration
          }
          break;
        case ' ':
          setActiveKey(' ');
          clearInterval(accelerateInterval);
          clearInterval(decelerateInterval);
          setSpeed(0);
          sendCommand(`${COMMAND.INCREASE_SPEED}-0`); // Emergency stop
          break;
        case '0':
          setActiveKey('0');
          sendCommand(COMMAND.CMD_BUZZER);
          break;
        default:
          break;
      }
    }, 300);

    const handleKeyUp = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          clearInterval(accelerateInterval);
          accelerateInterval = undefined;
          setActiveKey(null);
          break;
        case 'o':
        case 'O':
          clearInterval(decelerateInterval);
          decelerateInterval = undefined;
          setActiveKey(null);
          break;
        case ' ':
          setActiveKey(null);
          break;
        case '0':
          setActiveKey(null);
          sendCommand(COMMAND.CMD_BUZZER_STOP); // Send stop command for buzzer
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
  }, [sendCommand]);

  const getProgressColor = () => {
    if (speed <= 20) return 'bg-red-500';
    if (speed <= 60) return 'bg-yellow-500';
    return 'bg-green-500';
  };

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
          onClick={() => {
            clearInterval(accelerateInterval);
            clearInterval(decelerateInterval);
            setSpeed(0);
            sendCommand(`${COMMAND.INCREASE_SPEED}-0`);
          }}
        >
          Space (E-stop)
        </button>
        <button
          className={`w-32 h-12 rounded-lg ${getButtonClass('0')} text-white mt-4 flex items-center justify-center`}
          onMouseDown={() => sendCommand(COMMAND.CMD_BUZZER)}
          onMouseUp={() => sendCommand(COMMAND.CMD_BUZZER_STOP)}
        >
          0 (Buzz)
        </button>
      </div>
    </div>
  );
};

export default SpeedControl;
