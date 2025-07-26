/*
# File: /src/components/control/CameraControlPanel.tsx
# Summary:
This component controls the robot's camera (servo) movements using the CommandContext WebSocket connection.
It sends servo angle adjustment commands based on user input—both keyboard (arrow keys) and button click interactions.
*/

import React, { useState, useCallback, useEffect } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';

type ButtonDirection = "up" | "down" | "left" | "right";

const CameraControlPanel: React.FC = () => {
  const [buttonState, setButtonState] = useState<Record<ButtonDirection, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  const { sendCommand, addCommand } = useCommand();

  // Handle key press events for controlling the camera
  const handleKeyDown = useCallback((event: KeyboardEvent) => {
    switch (event.key) {
      case 'ArrowUp':
        sendCommand(COMMAND.CMD_SERVO_VERTICAL, { angle: 10 });
        setButtonPressed("up", true);
        addCommand('camera-up');
        break;
      case 'ArrowDown':
        sendCommand(COMMAND.CMD_SERVO_VERTICAL, { angle: -10 });
        setButtonPressed("down", true);
        addCommand('camera-down');
        break;
      case 'ArrowLeft':
        sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, { angle: 10 });
        setButtonPressed("left", true);
        addCommand('camera-left');
        break;
      case 'ArrowRight':
        sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, { angle: -10 });
        setButtonPressed("right", true);
        addCommand('camera-right');
        break;
      default:
        break;
    }
  }, [sendCommand, addCommand]);

  // Handle key release events for button UI feedback
  const handleKeyUp = useCallback((event: KeyboardEvent) => {
    switch (event.key) {
      case 'ArrowUp': setButtonPressed("up", false); break;
      case 'ArrowDown': setButtonPressed("down", false); break;
      case 'ArrowLeft': setButtonPressed("left", false); break;
      case 'ArrowRight': setButtonPressed("right", false); break;
      default: break;
    }
  }, []);

  // Add event listeners for key presses/releases
  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKeyDown, handleKeyUp]);

  // Update button state for UI feedback
  const setButtonPressed = (direction: ButtonDirection, state: boolean) => {
    setButtonState((prev) => ({ ...prev, [direction]: state }));
  };

  // Generate CSS classes for button styles based on state
  const buttonClass = (direction: ButtonDirection) =>
    `bg-gray-800 text-white p-4 m-1 rounded-lg ${buttonState[direction] ? 'bg-gray-600' : 'bg-gray-800'}`;

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Camera Control</div>
      <button
        className={buttonClass("up")}
        onMouseDown={() => { sendCommand(COMMAND.CMD_SERVO_VERTICAL, { angle: 10 }); setButtonPressed("up", true); }}
        onMouseUp={() => setButtonPressed("up", false)}
        onMouseLeave={() => setButtonPressed("up", false)}
      >
        ↑
      </button>
      <div className="flex space-x-1">
        <button
          className={buttonClass("left")}
          onMouseDown={() => { sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, { angle: 10 }); setButtonPressed("left", true); }}
          onMouseUp={() => setButtonPressed("left", false)}
          onMouseLeave={() => setButtonPressed("left", false)}
        >
          ←
        </button>
        <button
          className={buttonClass("right")}
          onMouseDown={() => { sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, { angle: -10 }); setButtonPressed("right", true); }}
          onMouseUp={() => setButtonPressed("right", false)}
          onMouseLeave={() => setButtonPressed("right", false)}
        >
          →
        </button>
      </div>
      <button
        className={buttonClass("down")}
        onMouseDown={() => { sendCommand(COMMAND.CMD_SERVO_VERTICAL, { angle: -10 }); setButtonPressed("down", true); }}
        onMouseUp={() => setButtonPressed("down", false)}
        onMouseLeave={() => setButtonPressed("down", false)}
      >
        ↓
      </button>
    </div>
  );
};

export default CameraControlPanel;
