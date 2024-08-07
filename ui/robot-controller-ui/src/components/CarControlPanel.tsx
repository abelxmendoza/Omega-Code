import React, { useState, useEffect, useRef } from 'react';
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
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.command) {
        addCommand(data.command);
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Cleanup on unmount
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, [addCommand]);

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
        sendCommand(COMMAND.STOP);
        break;
      case 'a':
      case 'A':
        setButtonPressed('left', false);
        sendCommand(COMMAND.STOP);
        break;
      case 's':
      case 'S':
        setButtonPressed('down', false);
        sendCommand(COMMAND.STOP);
        break;
      case 'd':
      case 'D':
        setButtonPressed('right', false);
        sendCommand(COMMAND.STOP);
        break;
      default:
        break;
    }
  };

  useEffect(() => {
    if (typeof window !== 'undefined') {
      window.addEventListener('keydown', handleKeyDown);
      window.addEventListener('keyup', handleKeyUp);
      return () => {
        window.removeEventListener('keydown', handleKeyDown);
        window.removeEventListener('keyup', handleKeyUp);
      };
    }
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
    sendCommand(COMMAND.STOP);
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
