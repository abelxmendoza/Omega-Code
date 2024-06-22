// File: /Omega-Code/ui/robot-controller-ui/src/components/LedControl.tsx

/*
This component allows the user to control the LED settings of the robot.
It includes options to select the color, mode, pattern, and interval for the LED.
The selected settings are sent as a command to the server.
*/

import React, { useState } from 'react';
import { SketchPicker } from 'react-color';

const LedControl: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
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
      interval,
    }));
  };

  return (
    <div className="bg-gray-900 p-4 rounded-lg shadow-md w-full md:w-3/4 lg:w-1/2">
      <h2 className="text-lg font-bold text-green-400 border-b-2 border-green-400 pb-2">LED Control</h2>
      <div className="mt-4">
        <SketchPicker color={color} onChange={handleColorChange} /> {/* Color picker component */}
        <div className="mt-4">
          <label className="block text-green-300">Mode:</label>
          <select value={mode} onChange={handleModeChange} className="w-full p-2 rounded bg-gray-800 text-green-300"> {/* Mode dropdown */}
            <option value="single">Single Color</option>
            <option value="multi">Multi Color</option>
            <option value="two">Two Color</option>
          </select>
        </div>
        <div className="mt-4">
          <label className="block text-green-300">Pattern:</label>
          <select value={pattern} onChange={handlePatternChange} className="w-full p-2 rounded bg-gray-800 text-green-300"> {/* Pattern dropdown */}
            <option value="static">Static</option>
            <option value="blink">Blink</option>
            <option value="fade">Fade</option>
          </select>
        </div>
        <div className="mt-4">
          <label className="block text-green-300">Interval (ms):</label>
          <input type="number" value={interval} onChange={handleIntervalChange} className="w-full p-2 rounded bg-gray-800 text-green-300" /> {/* Interval input */}
        </div>
        <button onClick={handleApply} className="mt-4 w-full bg-green-500 p-2 rounded text-white">Apply</button> {/* Apply button */}
      </div>
    </div>
  );
};

export default LedControl;
