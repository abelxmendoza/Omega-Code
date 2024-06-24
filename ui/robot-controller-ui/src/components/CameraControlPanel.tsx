import React from 'react';
import { COMMAND } from '../control_definitions';

const CameraControlPanel: React.FC<{ sendCommand: (command: string, angle: number) => void }> = ({ sendCommand }) => {
  const [buttonState, setButtonState] = React.useState({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  const handleKeyDown = (event: KeyboardEvent) => {
    switch (event.key) {
      case 'ArrowUp':
        sendCommand(COMMAND.CMD_SERVO_VERTICAL, 10);
        setButtonPressed('up', true);
        break;
      case 'ArrowLeft':
        sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, 10);
        setButtonPressed('left', true);
        break;
      case 'ArrowDown':
        sendCommand(COMMAND.CMD_SERVO_VERTICAL, -10);
        setButtonPressed('down', true);
        break;
      case 'ArrowRight':
        sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, -10);
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

  React.useEffect(() => {
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

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Camera Control</div>
      <button
        className={buttonClass('up')}
        onClick={() => sendCommand(COMMAND.CMD_SERVO_VERTICAL, 10)}
        onMouseDown={() => setButtonPressed('up', true)}
        onMouseUp={() => setButtonPressed('up', false)}
        onMouseLeave={() => setButtonPressed('up', false)}
      >
        ↑
      </button>
      <div className="flex space-x-1">
        <button
          className={buttonClass('left')}
          onClick={() => sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, 10)}
          onMouseDown={() => setButtonPressed('left', true)}
          onMouseUp={() => setButtonPressed('left', false)}
          onMouseLeave={() => setButtonPressed('left', false)}
        >
          ←
        </button>
        <button
          className={buttonClass('right')}
          onClick={() => sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, -10)}
          onMouseDown={() => setButtonPressed('right', true)}
          onMouseUp={() => setButtonPressed('right', false)}
          onMouseLeave={() => setButtonPressed('right', false)}
        >
          →
        </button>
      </div>
      <button
        className={buttonClass('down')}
        onClick={() => sendCommand(COMMAND.CMD_SERVO_VERTICAL, -10)}
        onMouseDown={() => setButtonPressed('down', true)}
        onMouseUp={() => setButtonPressed('down', false)}
        onMouseLeave={() => setButtonPressed('down', false)}
      >
        ↓
      </button>
    </div>
  );
};

export default CameraControlPanel;
