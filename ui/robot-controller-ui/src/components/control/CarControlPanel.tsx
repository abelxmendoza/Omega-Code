/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/control/CarControlPanel.tsx
# Summary:
Provides directional controls for the robot with keyboard and mouse inputs.
Commands are sent to the robot via WebSocket and logged in the CommandContext.
Includes visual feedback for user interactions and command logging.
*/

import React, { useState, useCallback, useEffect } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';

// Allowed directions for movement
const directions = ['up', 'down', 'left', 'right'] as const;
type Direction = typeof directions[number];

const keyToDirection: Record<string, Direction> = {
  w: 'up',
  a: 'left',
  s: 'down',
  d: 'right',
};

const directionToCommand: Record<Direction, string> = {
  up: COMMAND.MOVE_UP,
  down: COMMAND.MOVE_DOWN,
  left: COMMAND.MOVE_LEFT,
  right: COMMAND.MOVE_RIGHT,
};

const CarControlPanel: React.FC = () => {
  const [buttonState, setButtonState] = useState<Record<Direction, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  const { sendCommand, addCommand } = useCommand();

  const setButtonPressed = (direction: Direction, state: boolean) => {
    setButtonState((prevState) => ({ ...prevState, [direction]: state }));
  };

  const handleButtonClick = (command: string, direction: Direction) => {
    sendCommand(command);
    addCommand(`Command Sent: ${command}`);
    setButtonPressed(direction, true);
  };

  const handleButtonRelease = (direction: Direction) => {
    sendCommand(COMMAND.MOVE_STOP);
    addCommand(`Command Sent: ${COMMAND.MOVE_STOP}`);
    setButtonPressed(direction, false);
  };

  // --- Keyboard support for WASD ---
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key in keyToDirection) {
        const direction = keyToDirection[key];
        if (!buttonState[direction]) {
          handleButtonClick(directionToCommand[direction], direction);
        }
        event.preventDefault();
      }
    },
    [buttonState]
  );

  const handleKeyUp = useCallback(
    (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key in keyToDirection) {
        const direction = keyToDirection[key];
        handleButtonRelease(direction);
        event.preventDefault();
      }
    },
    []
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKeyDown, handleKeyUp]);

  const buttonClass = (direction: Direction) =>
    `p-4 m-1 rounded-lg text-white transition-colors duration-100 ${
      buttonState[direction] ? 'bg-red-600' : 'bg-gray-800'
    }`;

  return (
    <div className="flex flex-col items-center">
      <h2 className="text-lg font-bold mb-2">Car Control</h2>
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
