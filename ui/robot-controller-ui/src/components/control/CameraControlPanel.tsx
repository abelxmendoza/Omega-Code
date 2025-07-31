/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/control/CameraControlPanel.tsx
# Summary:
This component provides UI and keyboard control for the robot's camera (pan/tilt) servos.
It sends explicit directional commands (up, down, left, right) to the backend,
allowing precise nudge control via WebSocket for both button and arrow key events.
*/

import React, { useState, useCallback, useEffect } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';

type ButtonDirection = 'up' | 'down' | 'left' | 'right';

const CameraControlPanel: React.FC = () => {
  const [buttonState, setButtonState] = useState<Record<ButtonDirection, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false,
  });

  const { sendCommand, addCommand } = useCommand();

  // Handle key press events for controlling the camera
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      switch (event.key) {
        case 'ArrowUp':
          sendCommand(COMMAND.CAMERA_SERVO_UP);
          setButtonPressed('up', true);
          addCommand('camera-up');
          break;
        case 'ArrowDown':
          sendCommand(COMMAND.CAMERA_SERVO_DOWN);
          setButtonPressed('down', true);
          addCommand('camera-down');
          break;
        case 'ArrowLeft':
          sendCommand(COMMAND.CAMERA_SERVO_LEFT);
          setButtonPressed('left', true);
          addCommand('camera-left');
          break;
        case 'ArrowRight':
          sendCommand(COMMAND.CAMERA_SERVO_RIGHT);
          setButtonPressed('right', true);
          addCommand('camera-right');
          break;
        default:
          break;
      }
    },
    [sendCommand, addCommand]
  );

  const handleKeyUp = useCallback((event: KeyboardEvent) => {
    switch (event.key) {
      case 'ArrowUp':
        setButtonPressed('up', false);
        break;
      case 'ArrowDown':
        setButtonPressed('down', false);
        break;
      case 'ArrowLeft':
        setButtonPressed('left', false);
        break;
      case 'ArrowRight':
        setButtonPressed('right', false);
        break;
      default:
        break;
    }
  }, []);

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKeyDown, handleKeyUp]);

  const setButtonPressed = (direction: ButtonDirection, state: boolean) => {
    setButtonState((prev) => ({ ...prev, [direction]: state }));
  };

  // Change color to red when pressed
  const buttonClass = (direction: ButtonDirection) =>
    `p-4 m-1 rounded-lg text-white transition-colors duration-100 ${
      buttonState[direction] ? 'bg-red-600' : 'bg-gray-800'
    }`;

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Camera Control</div>
      <button
        className={buttonClass('up')}
        onMouseDown={() => {
          sendCommand(COMMAND.CAMERA_SERVO_UP);
          setButtonPressed('up', true);
        }}
        onMouseUp={() => setButtonPressed('up', false)}
        onMouseLeave={() => setButtonPressed('up', false)}
      >
        ↑
      </button>
      <div className="flex space-x-1">
        <button
          className={buttonClass('left')}
          onMouseDown={() => {
            sendCommand(COMMAND.CAMERA_SERVO_LEFT);
            setButtonPressed('left', true);
          }}
          onMouseUp={() => setButtonPressed('left', false)}
          onMouseLeave={() => setButtonPressed('left', false)}
        >
          ←
        </button>
        <button
          className={buttonClass('right')}
          onMouseDown={() => {
            sendCommand(COMMAND.CAMERA_SERVO_RIGHT);
            setButtonPressed('right', true);
          }}
          onMouseUp={() => setButtonPressed('right', false)}
          onMouseLeave={() => setButtonPressed('right', false)}
        >
          →
        </button>
      </div>
      <button
        className={buttonClass('down')}
        onMouseDown={() => {
          sendCommand(COMMAND.CAMERA_SERVO_DOWN);
          setButtonPressed('down', true);
        }}
        onMouseUp={() => setButtonPressed('down', false)}
        onMouseLeave={() => setButtonPressed('down', false)}
      >
        ↓
      </button>
    </div>
  );
};

export default CameraControlPanel;
