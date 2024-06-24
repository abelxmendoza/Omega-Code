import React from 'react';
import { COMMAND } from '../control_definitions';

const CarControlPanel: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  const [buttonState, setButtonState] = React.useState({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  const handleKeyDown = (event: KeyboardEvent) => {
    switch (event.key) {
      case 'w':
      case 'W':
        sendCommand(COMMAND.MOVE_UP);
        setButtonPressed('up', true);
        break;
      case 'a':
      case 'A':
        sendCommand(COMMAND.MOVE_LEFT);
        setButtonPressed('left', true);
        break;
      case 's':
      case 'S':
        sendCommand(COMMAND.MOVE_DOWN);
        setButtonPressed('down', true);
        break;
      case 'd':
      case 'D':
        sendCommand(COMMAND.MOVE_RIGHT);
        setButtonPressed('right', true);
        break;
      default:
        break;
    }
  };

  const handleKeyUp = (event: KeyboardEvent) => {
    switch (event.key) {
      case 'w':
      case 'W':
        setButtonPressed('up', false);
        break;
      case 'a':
      case 'A':
        setButtonPressed('left', false);
        break;
      case 's':
      case 'S':
        setButtonPressed('down', false);
        break;
      case 'd':
      case 'D':
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
    return `bg-gray-800 text-white p-2 rounded ${
      buttonState[direction] ? 'bg-gray-600' : 'bg-gray-800'
    }`;
  };

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Car Control</div>
      <button
        className={buttonClass('up')}
        onClick={() => sendCommand(COMMAND.MOVE_UP)}
        onMouseDown={() => setButtonPressed('up', true)}
        onMouseUp={() => setButtonPressed('up', false)}
        onMouseLeave={() => setButtonPressed('up', false)}
      >
        W
      </button>
      <div className="flex space-x-1">
        <button
          className={buttonClass('left')}
          onClick={() => sendCommand(COMMAND.MOVE_LEFT)}
          onMouseDown={() => setButtonPressed('left', true)}
          onMouseUp={() => setButtonPressed('left', false)}
          onMouseLeave={() => setButtonPressed('left', false)}
        >
          A
        </button>
        <button
          className={buttonClass('right')}
          onClick={() => sendCommand(COMMAND.MOVE_RIGHT)}
          onMouseDown={() => setButtonPressed('right', true)}
          onMouseUp={() => setButtonPressed('right', false)}
          onMouseLeave={() => setButtonPressed('right', false)}
        >
          D
        </button>
      </div>
      <button
        className={buttonClass('down')}
        onClick={() => sendCommand(COMMAND.MOVE_DOWN)}
        onMouseDown={() => setButtonPressed('down', true)}
        onMouseUp={() => setButtonPressed('down', false)}
        onMouseLeave={() => setButtonPressed('down', false)}
      >
        S
      </button>
    </div>
  );
};

export default CarControlPanel;
