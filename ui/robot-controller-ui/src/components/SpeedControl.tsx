/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/SpeedControl.tsx
# Summary:
This component provides a UI for controlling the robot's speed and additional features such as activating LEDs and a buzzer.
It handles speed changes, emergency stops, and toggling LED states. Commands are sent to the backend via WebSocket.
*/

import React, { useState, useEffect, useContext } from 'react';
import { COMMAND } from '../control_definitions';
import { CommandLogContext } from './CommandLogContext';

interface SpeedControlProps {
  sendCommand: (command: string) => void; // Function to send commands to the backend
  onOpenLedModal: () => void; // Function to open the LED configuration modal
}

const SpeedControl: React.FC<SpeedControlProps> = ({ sendCommand, onOpenLedModal }) => {
  const commandLogContext = useContext(CommandLogContext);
  if (!commandLogContext) {
    throw new Error('CommandLogContext must be used within its provider');
  }
  const { addCommand } = commandLogContext;

  const [speed, setSpeed] = useState(0);
  const [isLedActive, setIsLedActive] = useState(false);

  const increaseSpeed = () => {
    const newSpeed = Math.min(speed + 10, 100);
    setSpeed(newSpeed);
    sendCommand(`${COMMAND.INCREASE_SPEED}-${newSpeed}`);
    addCommand(`Speed increased to ${newSpeed}%`);
  };

  const decreaseSpeed = () => {
    const newSpeed = Math.max(speed - 10, 0);
    setSpeed(newSpeed);
    sendCommand(`${COMMAND.DECREASE_SPEED}-${newSpeed}`);
    addCommand(`Speed decreased to ${newSpeed}%`);
  };

  const emergencyStop = () => {
    setSpeed(0);
    sendCommand('emergency-stop');
    addCommand('Emergency stop activated');
  };

  const toggleLed = () => {
    setIsLedActive(!isLedActive);
    sendCommand('toggle-led');
    addCommand('LED toggled');
    onOpenLedModal();
  };

  const activateBuzzer = () => {
    sendCommand(COMMAND.CMD_BUZZER);
    addCommand('Buzzer activated');
    setTimeout(() => sendCommand(COMMAND.CMD_BUZZER_STOP), 5000); // Stop buzzer after 5 seconds
  };

  // Keyboard event handlers
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p': // Gas pedal
          increaseSpeed();
          break;
        case 'o': // Brake
          decreaseSpeed();
          break;
        case ' ': // Emergency stop
          emergencyStop();
          break;
        case 'i': // LED toggle
          toggleLed();
          break;
        case '0': // Buzzer
          activateBuzzer();
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [speed, isLedActive]); // Dependencies ensure the latest state is used

  return (
    <div className="bg-gray-800 text-white p-3 rounded-md shadow-md w-full max-w-sm flex flex-col items-center">
      <h2 className="text-md font-bold mb-3">Speed Control</h2>

      {/* Speed Progress Bar */}
      <div className="w-full mb-3">
        <label className="block text-sm font-semibold mb-1">Speed:</label>
        <div className="w-full bg-gray-300 rounded h-3 relative">
          <div
            className={`h-full rounded ${
              speed <= 20 ? 'bg-red-500' : speed <= 60 ? 'bg-yellow-500' : 'bg-green-500'
            }`}
            style={{ width: `${speed}%` }}
          ></div>
          <div className="absolute inset-0 flex justify-center items-center text-white text-xs font-bold">
            {speed}%
          </div>
        </div>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-2 w-full">
        <button
          className="bg-blue-600 text-white py-1 rounded hover:bg-blue-700 flex flex-col items-center"
          onClick={decreaseSpeed}
        >
          <span>O</span>
          <span>(Brake)</span>
        </button>
        <button
          className="bg-blue-600 text-white py-1 rounded hover:bg-blue-700 flex flex-col items-center"
          onClick={increaseSpeed}
        >
          <span>P</span>
          <span>(Gas)</span>
        </button>
        <button
          className="bg-red-600 text-white py-1 rounded hover:bg-red-700 flex flex-col items-center col-span-2"
          onClick={emergencyStop}
        >
          <span>Space</span>
          <span>(Stop)</span>
        </button>
        <button
          className="bg-yellow-600 text-white py-1 rounded hover:bg-yellow-700 flex flex-col items-center"
          onDoubleClick={toggleLed}
        >
          <span>I</span>
          <span>(LED)</span>
        </button>
        <button
          className="bg-purple-600 text-white py-1 rounded hover:bg-purple-700 flex flex-col items-center"
          onClick={activateBuzzer}
        >
          <span>0</span>
          <span>(Buzzer)</span>
        </button>
      </div>
    </div>
  );
};

export default SpeedControl;
