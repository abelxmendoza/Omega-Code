// File: /Omega-Code/ui/robot-controller-ui/src/components/LedModal.tsx

/*
This component provides a modal interface for controlling the LED settings of the robot.
It includes options to select the color, mode, pattern, and interval for the LED.
The selected settings are sent as a command to the server.
*/

import React, { useState } from 'react';
import { SketchPicker } from 'react-color';

const LedModal: React.FC<{ sendCommand: (command: string) => void, isOpen: boolean, onClose: () => void }> = ({ sendCommand, isOpen, onClose }) => {
  const [color, setColor] = useState('#ffffff'); // State to store the selected color
  const [mode, setMode] = useState('single'); // State to store the selected mode
  const [pattern, setPattern] = useState('static'); // State to store the selected pattern
  const [interval, setInterval] = useState(1000); // State to store the interval in milliseconds

  // Handle color change from the color picker
  const handleColorChange = (color: any) => {
    setColor(color.hex);
  };

  // Handle mode change from the dropdown
  const handleModeChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setMode(event.target.value);
  };

  // Handle pattern change from the dropdown
  const handlePatternChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setPattern(event.target.value);
  };

  // Handle interval change from the input field
  const handleIntervalChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInterval(Number(event.target.value));
  };

  // Handle apply button click to send the command with the selected settings
  const handleApply = () => {
    sendCommand(JSON.stringify({
      command: 'set-led',
      color,
      mode,
      pattern,
      interval: pattern !== 'static' ? interval : undefined, // Only include interval if pattern is not static
    }));
  };

  // Do not render the modal if it is not open
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-gray-800 bg-opacity-75 flex justify-center items-center z-50">
      <div className="bg-gray-900 rounded-lg p-4 w-full max-w-lg relative text-white">
        <button className="absolute top-0 right-0 m-4 text-white" onClick={onClose}>X</button>
        <p className="text-sm text-gray-400 font-bold mb-4">⚠️ Double-click the "I" key to turn the lights on.</p> {/* Instruction text */}
        <SketchPicker color={color} onChange={handleColorChange} /> {/* Color picker component */}
        <div>
          <label className="block mt-4">Mode:</label>
          <select value={mode} onChange={handleModeChange} className="bg-gray-800 text-white rounded p-1"> {/* Mode dropdown */}
            <option value="single">Single Color</option>
            <option value="multi">Multi Color</option>
            <option value="two">Two Color</option>
          </select>
        </div>
        <div>
          <label className="block mt-4">Pattern:</label>
          <select value={pattern} onChange={handlePatternChange} className="bg-gray-800 text-white rounded p-1"> {/* Pattern dropdown */}
            <option value="static">Static</option>
            <option value="blink">Blink</option>
            <option value="fade">Fade</option>
          </select>
        </div>
        {pattern !== 'static' && ( // Show interval input only if pattern is not static
          <div>
            <label className="block mt-4">Interval (ms):</label>
            <input type="number" value={interval} onChange={handleIntervalChange} className="bg-gray-800 text-white rounded p-1" /> {/* Interval input */}
          </div>
        )}
        <button onClick={handleApply} className="bg-blue-500 text-white p-2 rounded mt-4">Apply</button> {/* Apply button */}
      </div>
    </div>
  );
};

export default LedModal;
