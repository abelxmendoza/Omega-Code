/*
# File: /src/components/CameraControlPanel.tsx
# Summary:
This component controls the robot's camera movements using a WebSocket connection.
It sends servo angle adjustment commands based on user input, including arrow key and mouse interactions.
*/

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { COMMAND } from '../control_definitions';
import { useCommandLog } from './CommandLogContext';

// Define the type for button state keys
type ButtonDirection = "up" | "down" | "left" | "right";

const CameraControlPanel: React.FC<{ sendCommand: (command: string, angle: number) => void }> = ({ sendCommand }) => {
  const [buttonState, setButtonState] = useState<Record<ButtonDirection, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false,
  });
  const { addCommand } = useCommandLog();
  const ws = useRef<WebSocket | null>(null);
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://localhost:8080/ws';

  // Establish WebSocket connection
  useEffect(() => {
    ws.current = new WebSocket(wsUrl);

    ws.current.onopen = () => console.log('WebSocket connection established');
    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.command) {
        addCommand(data.command);
      }
    };
    ws.current.onclose = () => console.log('WebSocket connection closed');
    ws.current.onerror = (error) => console.error('WebSocket error:', error);

    // Clean up WebSocket connection on unmount
    return () => ws.current?.close();
  }, [addCommand, wsUrl]);

  // Handle key press events for controlling the camera
  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      switch (event.key) {
        case 'ArrowUp':
          sendCommand(COMMAND.CMD_SERVO_VERTICAL, 10);
          setButtonPressed("up", true);
          addCommand('camera-up');
          break;
        case 'ArrowDown':
          sendCommand(COMMAND.CMD_SERVO_VERTICAL, -10);
          setButtonPressed("down", true);
          addCommand('camera-down');
          break;
        case 'ArrowLeft':
          sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, 10);
          setButtonPressed("left", true);
          addCommand('camera-left');
          break;
        case 'ArrowRight':
          sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, -10);
          setButtonPressed("right", true);
          addCommand('camera-right');
          break;
        default:
          break;
      }
    },
    [sendCommand, addCommand]
  );

  // Handle key release events
  const handleKeyUp = useCallback(
    (event: KeyboardEvent) => {
      switch (event.key) {
        case 'ArrowUp':
          setButtonPressed("up", false);
          break;
        case 'ArrowDown':
          setButtonPressed("down", false);
          break;
        case 'ArrowLeft':
          setButtonPressed("left", false);
          break;
        case 'ArrowRight':
          setButtonPressed("right", false);
          break;
        default:
          break;
      }
    },
    []
  );

  // Add event listeners for key presses
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
    setButtonState((prevState) => ({ ...prevState, [direction]: state }));
  };

  // Generate CSS classes for button styles based on state
  const buttonClass = (direction: ButtonDirection) =>
    `bg-gray-800 text-white p-4 m-1 rounded-lg ${buttonState[direction] ? 'bg-gray-600' : 'bg-gray-800'}`;

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Camera Control</div>
      <button className={buttonClass("up")} onMouseDown={() => sendCommand(COMMAND.CMD_SERVO_VERTICAL, 10)}>
        ↑
      </button>
      <div className="flex space-x-1">
        <button className={buttonClass("left")} onMouseDown={() => sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, 10)}>
          ←
        </button>
        <button className={buttonClass("right")} onMouseDown={() => sendCommand(COMMAND.CMD_SERVO_HORIZONTAL, -10)}>
          →
        </button>
      </div>
      <button className={buttonClass("down")} onMouseDown={() => sendCommand(COMMAND.CMD_SERVO_VERTICAL, -10)}>
        ↓
      </button>
    </div>
  );
};

export default CameraControlPanel;
