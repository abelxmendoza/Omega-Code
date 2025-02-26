/*
# File: /src/components/control/CarControlPanel.tsx
# Summary:
Provides directional controls for the robot with keyboard and mouse inputs.
Commands are sent to the robot via WebSocket and logged in the CommandContext.
Includes visual feedback for user interactions and command logging.
*/

import React, { useState } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';

// Allowed directions for movement
const directions = ['up', 'down', 'left', 'right'] as const;
type Direction = typeof directions[number];

const CarControlPanel: React.FC = () => {
  // Track pressed state of each directional button
  const [buttonState, setButtonState] = useState<Record<Direction, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  const { sendCommand, addCommand } = useCommand(); // Access WebSocket and command log functions

  /**
   * Updates the visual state of a button when pressed or released.
   * @param direction - The direction to update ('up', 'down', 'left', 'right').
   * @param state - The button state (true for pressed, false for released).
   */
  const setButtonPressed = (direction: Direction, state: boolean) => {
    setButtonState((prevState) => ({ ...prevState, [direction]: state }));
  };

  /**
   * Handles button press for movement. Logs and sends the corresponding movement command.
   * @param command - The movement command (e.g., MOVE_UP).
   * @param direction - The associated direction ('up', 'down', 'left', 'right').
   */
  const handleButtonClick = (command: string, direction: Direction) => {
    console.debug(`Button pressed: ${direction}, Command: ${command}`); // Debug log
    sendCommand(command); // Send movement command via WebSocket
    addCommand(`Command Sent: ${command}`); // Log command in CommandContext
    setButtonPressed(direction, true); // Update button visual state
  };

  /**
   * Handles button release to stop movement. Logs and sends the stop command.
   * @param direction - The direction of the button released ('up', 'down', 'left', 'right').
   */
  const handleButtonRelease = (direction: Direction) => {
    console.debug(`Button released: ${direction}, Command: ${COMMAND.MOVE_STOP}`); // Debug log
    sendCommand(COMMAND.MOVE_STOP); // Send stop command
    addCommand(`Command Sent: ${COMMAND.MOVE_STOP}`); // Log stop command
    setButtonPressed(direction, false); // Reset button visual state
  };

  /**
   * Generates a Tailwind CSS class string for a directional button.
   * Highlights the button when pressed.
   * @param direction - The direction of the button ('up', 'down', 'left', 'right').
   * @returns The CSS class string for the button.
   */
  const buttonClass = (direction: Direction) =>
    `bg-gray-800 text-white p-4 m-1 rounded-lg ${
      buttonState[direction] ? 'bg-gray-600' : 'bg-gray-800'
    }`;

  return (
    <div className="flex flex-col items-center">
      <h2 className="text-lg font-bold mb-2">Car Control</h2>

      {/* Button for moving up */}
      <button
        className={buttonClass('up')}
        onMouseDown={() => handleButtonClick(COMMAND.MOVE_UP, 'up')}
        onMouseUp={() => handleButtonRelease('up')}
        onMouseLeave={() => handleButtonRelease('up')}
      >
        W
      </button>

      {/* Buttons for moving left and right */}
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

      {/* Button for moving down */}
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
