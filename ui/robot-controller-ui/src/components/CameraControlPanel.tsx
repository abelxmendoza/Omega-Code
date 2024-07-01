import React, { useState, useEffect, useContext } from 'react';
import { COMMAND } from '../control_definitions';
import { useCommandLog } from './CommandLogContext';

const CameraControlPanel: React.FC<{ sendCommand: (command: string, angle: number) => void }> = ({ sendCommand }) => {
  const [buttonState, setButtonState] = useState({
    up: false,
    down: false,
    left: false,
    right: false,
  });
  const { addCommand } = useCommandLog();

  const handleKeyDown = (event: KeyboardEvent) => {
    switch (event.key) {
      case 'ArrowUp':
        sendCommand(COMMAND.CMD_SERVO_VERTICAL, 10);
        addCommand('camera-up');
        setButtonPressed('up', true);
        break;
      case 'ArrowLeft':
        sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, 10);
        addCommand('camera-left');
        setButtonPressed('left', true);
        break;
      case 'ArrowDown':
        sendCommand(COMMAND.CMD_SERVO_VERTICAL, -10);
        addCommand('camera-down');
        setButtonPressed('down', true);
        break;
      case 'ArrowRight':
        sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, -10);
        addCommand('camera-right');
        setButtonPressed('right', true);
        break;
      default:
        break;
    }
  };

  const handleKeyUp = (event: KeyboardEvent) => {
    switch (event.key) {
      case 'ArrowUp':
        setButtonPressed('up', false);
        break;
      case 'ArrowLeft':
        setButtonPressed('left', false);
        break;
      case 'ArrowDown':
        setButtonPressed('down', false);
        break;
      case 'ArrowRight':
        setButtonPressed('right', false);
        break;
      default:
        break;
    }
  };

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const setButtonPressed = (direction: string, state: boolean) => {
    setButtonState((prevState) => ({ ...prevState, [direction]: state }));
  };

  const buttonClass = (direction: string) => {
    return `bg-gray-800 text-white p-4 m-1 rounded-lg ${
      buttonState[direction] ? 'bg-gray-600' : 'bg-gray-800'
    }`;
  };

  const handleButtonClick = (command: string, angle: number, direction: string, logMessage: string) => {
    sendCommand(command, angle);
    addCommand(logMessage);
    setButtonPressed(direction, true);
  };

  const handleButtonRelease = (direction: string) => {
    setButtonPressed(direction, false);
  };

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Camera Control</div>
      <button
        className={buttonClass('up')}
        onMouseDown={() => handleButtonClick(COMMAND.CMD_SERVO_VERTICAL, 10, 'up', 'camera-up')}
        onMouseUp={() => handleButtonRelease('up')}
        onMouseLeave={() => handleButtonRelease('up')}
      >
        ↑
      </button>
      <div className="flex space-x-1">
        <button
          className={buttonClass('left')}
          onMouseDown={() => handleButtonClick(COMMAND.CMD_SERVO_HORIZONTAL, 10, 'left', 'camera-left')}
          onMouseUp={() => handleButtonRelease('left')}
          onMouseLeave={() => handleButtonRelease('left')}
        >
          ←
        </button>
        <button
          className={buttonClass('right')}
          onMouseDown={() => handleButtonClick(COMMAND.CMD_SERVO_HORIZONTAL, -10, 'right', 'camera-right')}
          onMouseUp={() => handleButtonRelease('right')}
          onMouseLeave={() => handleButtonRelease('right')}
        >
          →
        </button>
      </div>
      <button
        className={buttonClass('down')}
        onMouseDown={() => handleButtonClick(COMMAND.CMD_SERVO_VERTICAL, -10, 'down', 'camera-down')}
        onMouseUp={() => handleButtonRelease('down')}
        onMouseLeave={() => handleButtonRelease('down')}
      >
        ↓
      </button>
    </div>
  );
};

export default CameraControlPanel;
