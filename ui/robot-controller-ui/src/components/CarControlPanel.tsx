/*
# File: /src/components/CarControlPanel.tsx
# Summary:
This component provides reusable controls for the robot's movement using keyboard and mouse inputs.
It supports forward, backward, left, and right commands, making it adaptable for various projects.
*/

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { COMMAND } from '../control_definitions';
import { useCommandLog } from './CommandLogContext';

// Define allowed movement directions
const directions = ['up', 'down', 'left', 'right'] as const;
type Direction = typeof directions[number];

const CarControlPanel: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
  // State to track button press status for all directions
  const [buttonState, setButtonState] = useState<Record<Direction, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  // Access the command log context to log commands sent to the robot
  const { addCommand } = useCommandLog();

  // Ref for WebSocket connection
  const ws = useRef<WebSocket | null>(null);

  // WebSocket URL from environment variable, with a default fallback
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://localhost:8080/ws';

  // Establish WebSocket connection and handle events
  useEffect(() => {
    ws.current = new WebSocket(wsUrl);

    ws.current.onopen = () => console.log('WebSocket connection established');
    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.command) {
        // Log received commands (if needed)
        addCommand(data.command);
      }
    };
    ws.current.onclose = () => console.log('WebSocket connection closed');
    ws.current.onerror = (error) => console.error('WebSocket error:', error);

    return () => ws.current?.close();
  }, [addCommand, wsUrl]);

  // Handle keydown events for robot movement
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'w': // Move forward
          sendCommand(COMMAND.MOVE_UP);
          setButtonPressed('up', true);
          addCommand(COMMAND.MOVE_UP);
          break;
        case 'a': // Move left
          sendCommand(COMMAND.MOVE_LEFT);
          setButtonPressed('left', true);
          addCommand(COMMAND.MOVE_LEFT);
          break;
        case 's': // Move backward
          sendCommand(COMMAND.MOVE_DOWN);
          setButtonPressed('down', true);
          addCommand(COMMAND.MOVE_DOWN);
          break;
        case 'd': // Move right
          sendCommand(COMMAND.MOVE_RIGHT);
          setButtonPressed('right', true);
          addCommand(COMMAND.MOVE_RIGHT);
          break;
        default:
          break;
      }
    },
    [sendCommand, addCommand]
  );

  // Handle keyup events to stop the robot's movement
  const handleKeyUp = useCallback(
    (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'w': // Stop forward movement
          setButtonPressed('up', false);
          sendCommand(COMMAND.MOVE_STOP);
          break;
        case 'a': // Stop left movement
          setButtonPressed('left', false);
          sendCommand(COMMAND.MOVE_STOP);
          break;
        case 's': // Stop backward movement
          setButtonPressed('down', false);
          sendCommand(COMMAND.MOVE_STOP);
          break;
        case 'd': // Stop right movement
          setButtonPressed('right', false);
          sendCommand(COMMAND.MOVE_STOP);
          break;
        default:
          break;
      }
    },
    [sendCommand]
  );

  // Attach keydown and keyup listeners to the window
  useEffect(() => {
    if (typeof window !== 'undefined') {
      window.addEventListener('keydown', handleKeyDown);
      window.addEventListener('keyup', handleKeyUp);
      return () => {
        window.removeEventListener('keydown', handleKeyDown);
        window.removeEventListener('keyup', handleKeyUp);
      };
    }
  }, [handleKeyDown, handleKeyUp]);

  // Update the button press state for visual feedback
  const setButtonPressed = (direction: Direction, state: boolean) => {
    setButtonState((prevState) => ({ ...prevState, [direction]: state }));
  };

  // Get the CSS class for buttons based on their pressed state
  const buttonClass = (direction: Direction) =>
    `bg-gray-800 text-white p-4 m-1 rounded-lg ${buttonState[direction] ? 'bg-gray-600' : 'bg-gray-800'}`;

  // Handle mouse button clicks for robot movement
  const handleButtonClick = (command: string, direction: Direction) => {
    sendCommand(command);
    addCommand(command);
    setButtonPressed(direction, true);
  };

  // Handle mouse button release to stop movement
  const handleButtonRelease = (direction: Direction) => {
    setButtonPressed(direction, false);
    sendCommand(COMMAND.MOVE_STOP);
  };

  return (
    <div className="flex flex-col items-center">
      {/* Header */}
      <div className="text-lg font-bold mb-2">Car Control</div>

      {/* Movement Control Buttons */}
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
