import React, { useState, useEffect, useRef } from 'react';
import { COMMAND } from '../control_definitions';

interface SpeedControlProps {
  sendCommand: (command: string) => void;
  onOpenLedModal: () => void;
}

const SpeedControl: React.FC<SpeedControlProps> = ({ sendCommand, onOpenLedModal }) => {
  const [speed, setSpeed] = useState(0);
  const [activeKey, setActiveKey] = useState<string | null>(null);
  const accelerateInterval = useRef<NodeJS.Timeout | undefined>(undefined);
  const decelerateInterval = useRef<NodeJS.Timeout | undefined>(undefined);
  const buzzTimeout = useRef<NodeJS.Timeout | undefined>(undefined);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          if (!accelerateInterval.current) {
            setActiveKey('p');
            accelerateInterval.current = setInterval(() => {
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
          if (!decelerateInterval.current) {
            setActiveKey('o');
            decelerateInterval.current = setInterval(() => {
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
          clearInterval(accelerateInterval.current);
          clearInterval(decelerateInterval.current);
          setSpeed(0);
          sendCommand(`${COMMAND.INCREASE_SPEED}-0`); // Emergency stop
          break;
        case 'i':
        case 'I':
          setActiveKey('i');
          sendCommand('toggle-led');
          onOpenLedModal();
          break;
        default:
          break;
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      switch (event.key) {
        case 'p':
        case 'P':
          clearInterval(accelerateInterval.current);
          accelerateInterval.current = undefined;
          setActiveKey(null);
          break;
        case 'o':
        case 'O':
          clearInterval(decelerateInterval.current);
          decelerateInterval.current = undefined;
          setActiveKey(null);
          break;
        case ' ':
          setActiveKey(null);
          break;
        case 'i':
        case 'I':
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
      clearInterval(accelerateInterval.current);
      clearInterval(decelerateInterval.current);
    };
  }, [sendCommand, onOpenLedModal]);

  const getProgressColor = () => {
    if (speed <= 20) return 'bg-red-500';
    if (speed <= 60) return 'bg-yellow-500';
    return 'bg-green-500';
  };

  const getButtonClass = (key: string) => {
    return activeKey === key ? 'bg-red-500' : 'bg-blue-500';
  };

  const handleButtonClick = (command: string) => {
    console.log(`Button click: ${command}`);
    try {
      sendCommand(command);
    } catch (error) {
      console.error(`Error sending command ${command}:`, error);
    }
  };

  const handleButtonDoubleClick = (command: string) => {
    console.log(`Button double click: ${command}`);
    try {
      if (command === COMMAND.CMD_BUZZER) {
        sendCommand(command);
        buzzTimeout.current = setTimeout(() => {
          sendCommand(COMMAND.CMD_BUZZER_STOP);
        }, 10000); // Buzz for 10 seconds
      } else {
        sendCommand(command);
      }
    } catch (error) {
      console.error(`Error sending command ${command}:`, error);
    }
  };

  const handleButtonRelease = (command: string) => {
    console.log(`Button release: ${command}`);
    try {
      if (buzzTimeout.current) {
        clearTimeout(buzzTimeout.current);
        buzzTimeout.current = undefined;
      }
      sendCommand(command);
    } catch (error) {
      console.error(`Error sending command ${command}:`, error);
    }
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
            className={`w-16 h-16 rounded-lg ${getButtonClass('i')} text-white flex flex-col items-center justify-center`}
            onClick={() => {
              handleButtonClick('toggle-led');
              onOpenLedModal();
            }}
            onDoubleClick={() => handleButtonDoubleClick('toggle-led')}
          >
            <span>I</span>
            <span>(LED)</span>
          </button>
          <button
            className={`w-16 h-16 rounded-lg ${getButtonClass('o')} text-white flex flex-col items-center justify-center`}
            onClick={() => handleButtonClick(COMMAND.DECREASE_SPEED)}
            onDoubleClick={() => handleButtonDoubleClick(COMMAND.DECREASE_SPEED)}
          >
            <span>O</span>
            <span>(brake)</span>
          </button>
          <button
            className={`w-16 h-16 rounded-lg ${getButtonClass('p')} text-white flex flex-col items-center justify-center`}
            onClick={() => handleButtonClick(COMMAND.INCREASE_SPEED)}
            onDoubleClick={() => handleButtonDoubleClick(COMMAND.INCREASE_SPEED)}
          >
            <span>P</span>
            <span>(gas)</span>
          </button>
        </div>
        <button
          className={`w-32 h-12 rounded-lg ${getButtonClass(' ')} text-white mt-4 flex items-center justify-center`}
          onClick={() => {
            clearInterval(accelerateInterval.current);
            clearInterval(decelerateInterval.current);
            setSpeed(0);
            handleButtonClick(`${COMMAND.INCREASE_SPEED}-0`);
          }}
          onDoubleClick={() => handleButtonDoubleClick(`${COMMAND.INCREASE_SPEED}-0`)}
        >
          Space (E-stop)
        </button>
        <button
          className={`w-32 h-12 rounded-lg ${getButtonClass('0')} text-white mt-4 flex items-center justify-center`}
          onClick={() => handleButtonClick(COMMAND.CMD_BUZZER)}
          onDoubleClick={() => handleButtonDoubleClick(COMMAND.CMD_BUZZER)}
          onMouseUp={() => handleButtonRelease(COMMAND.CMD_BUZZER_STOP)}
        >
          0 (Buzz)
        </button>
      </div>
    </div>
  );
};

export default SpeedControl;
