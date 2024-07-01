import React, { useState, useEffect, useContext } from 'react';
import { COMMAND } from '../control_definitions';
import { useCommandLog } from './CommandLogContext';

const CarControlPanel: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  const [buttonState, setButtonState] = useState({
    up: false,
    down: false,
    left: false,
    right: false,
  });
  const { addCommand } = useCommandLog();

  const handleKeyDown = (event: KeyboardEvent) => {
    switch (event.key) {
      case 'w':
      case 'W':
        sendCommand(COMMAND.MOVE_UP);
        addCommand(COMMAND.MOVE_UP);
        setButtonPressed('up', true);
        break;
      case 'a':
      case 'A':
        sendCommand(COMMAND.MOVE_LEFT);
        addCommand(COMMAND.MOVE_LEFT);
        setButtonPressed('left', true);
        break;
      case 's':
      case 'S':
        sendCommand(COMMAND.MOVE_DOWN);
        addCommand(COMMAND.MOVE_DOWN);
        setButtonPressed('down', true);
        break;
      case 'd':
      case 'D':
        sendCommand(COMMAND.MOVE_RIGHT);
        addCommand(COMMAND.MOVE_RIGHT);
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

  const handleButtonClick = (command: string, direction: string) => {
    sendCommand(command);
    addCommand(command);
    setButtonPressed(direction, true);
  };

  const handleButtonRelease = (direction: string) => {
    setButtonPressed(direction, false);
  };

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Car Control</div>
      <button
        className={buttonClass('up')}
        onMouseDown={() => handleButtonClick(COMMAND.MOVE_UP, 'up')}
        onMouseUp={() => handleButtonRelease('up')}
        onMouseLeave={() => handleButtonRelease('up')}
      >
        W
      </button>
      <div className="flex space-x-1">
        <button
          className={buttonClass('left')}
          onMouseDown={() => handleButtonClick(COMMAND.MOVE_LEFT, 'left')}
          onMouseUp={() => handleButtonRelease('left')}
          onMouseLeave={() => handleButtonRelease('left')}
        >
          A
        </button>
        <button
          className={buttonClass('right')}
          onMouseDown={() => handleButtonClick(COMMAND.MOVE_RIGHT, 'right')}
          onMouseUp={() => handleButtonRelease('right')}
          onMouseLeave={() => handleButtonRelease('right')}
        >
          D
        </button>
      </div>
      <button
        className={buttonClass('down')}
        onMouseDown={() => handleButtonClick(COMMAND.MOVE_DOWN, 'down')}
        onMouseUp={() => handleButtonRelease('down')}
        onMouseLeave={() => handleButtonRelease('down')}
      >
        S
      </button>
    </div>
  );
};

export default CarControlPanel;
