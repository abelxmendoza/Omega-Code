/*
# File: /src/components/control/SpeedControl.tsx
# Summary:
Provides a user interface for controlling the robot's speed, LEDs, and buzzer. 
It includes controls for increasing/decreasing speed, performing an emergency stop, and toggling LED settings.
*/

import React, { useState, useEffect, useCallback } from 'react';
import { COMMAND } from '@/control_definitions'; // Import command definitions
import { useCommand } from '@/context/CommandContext'; // Context for WebSocket commands
import LedModal from '@/components/lighting/LedModal'; // Component for configuring LED settings

const SpeedControl: React.FC = () => {
  const { sendCommand } = useCommand(); // Access WebSocket command utilities
  const [speed, setSpeed] = useState(0); // State to track current speed (0-100%)
  const [isLedActive, setIsLedActive] = useState(false); // State for LED toggle
  const [isLedModalOpen, setIsLedModalOpen] = useState(false); // State for LED modal visibility

  /**
   * Increase the robot's speed (capped at 100%).
   */
  const increaseSpeed = useCallback(() => {
    const newSpeed = Math.min(speed + 10, 100);
    setSpeed(newSpeed);
    sendCommand(`${COMMAND.INCREASE_SPEED}-${newSpeed}`);
  }, [speed, sendCommand]);

  /**
   * Decrease the robot's speed (minimum of 0%).
   */
  const decreaseSpeed = useCallback(() => {
    const newSpeed = Math.max(speed - 10, 0);
    setSpeed(newSpeed);
    sendCommand(`${COMMAND.DECREASE_SPEED}-${newSpeed}`);
  }, [speed, sendCommand]);

  /**
   * Perform an emergency stop (speed set to 0%).
   */
  const emergencyStop = useCallback(() => {
    setSpeed(0);
    sendCommand(COMMAND.STOP);
  }, [sendCommand]);

  /**
   * Toggle the LED state and open the LED configuration modal.
   */
  const toggleLed = useCallback(() => {
    setIsLedActive((prev) => !prev);
    setIsLedModalOpen(true); // Open LED modal
  }, []);

  /**
   * Activate the buzzer for 5 seconds.
   */
  const activateBuzzer = useCallback(() => {
    sendCommand(COMMAND.CMD_BUZZER);
    setTimeout(() => sendCommand(COMMAND.CMD_BUZZER_STOP), 5000); // Stop buzzer after 5 seconds
  }, [sendCommand]);

  /**
   * Handle keyboard shortcuts for controls.
   */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p':
          increaseSpeed();
          break;
        case 'o':
          decreaseSpeed();
          break;
        case ' ':
          emergencyStop();
          break;
        case 'i':
          toggleLed();
          break;
        case '0':
          activateBuzzer();
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [increaseSpeed, decreaseSpeed, emergencyStop, toggleLed, activateBuzzer]);

  return (
    <div className="bg-gray-800 text-white p-4 rounded-md shadow-md w-full max-w-sm flex flex-col items-center">
      {/* Header */}
      <h2 className="text-lg font-bold mb-4">Speed & Light Control</h2>

      {/* Speed Progress Bar */}
      <div className="w-full mb-4">
        <label className="block text-sm font-semibold mb-2">Speed:</label>
        <div className="w-full bg-gray-300 rounded h-4 relative">
          <div
            className={`h-full rounded ${
              speed <= 20 ? 'bg-red-500' : speed <= 60 ? 'bg-yellow-500' : 'bg-green-500'
            }`}
            style={{ width: `${speed}%` }}
          ></div>
          <div className="absolute inset-0 flex justify-center items-center text-white text-sm font-bold">
            {speed}%
          </div>
        </div>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-3 w-full">
        <button
          className="bg-blue-600 text-white py-2 rounded hover:bg-blue-700"
          onClick={decreaseSpeed}
        >
          O (Brake)
        </button>
        <button
          className="bg-blue-600 text-white py-2 rounded hover:bg-blue-700"
          onClick={increaseSpeed}
        >
          P (Gas)
        </button>
        <button
          className="bg-red-600 text-white py-2 rounded hover:bg-red-700 col-span-2"
          onClick={emergencyStop}
        >
          Space (Stop)
        </button>
        <button
          className="bg-yellow-600 text-white py-2 rounded hover:bg-yellow-700 col-span-2"
          onClick={toggleLed}
        >
          I (LED)
        </button>
        <button
          className="bg-purple-600 text-white py-2 rounded hover:bg-purple-700 col-span-2"
          onClick={activateBuzzer}
        >
          0 (Buzzer)
        </button>
      </div>

      {/* LED Modal */}
      {isLedModalOpen && (
        <LedModal isOpen={isLedModalOpen} onClose={() => setIsLedModalOpen(false)} />
      )}
    </div>
  );
};

export default SpeedControl;
