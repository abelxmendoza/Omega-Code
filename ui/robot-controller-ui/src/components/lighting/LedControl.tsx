/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LedControl.tsx
# Summary:
Allows the user to configure LED settings for the robot, including color, mode, pattern, and interval.
Settings are sent as commands to the backend WebSocket server.
*/

import React, { useState } from 'react';
import { SketchPicker } from 'react-color';
import { COMMAND, LIGHTING_MODES, LIGHTING_PATTERNS } from '../../control_definitions';

// Props definition for the component
interface LedControlProps {
  sendCommand: (command: string, data?: any) => void; // Function to send commands to the backend
}

const LedControl: React.FC<LedControlProps> = ({ sendCommand }) => {
  // Component states for managing LED settings
  const [color, setColor] = useState('#ffffff'); // Stores the selected color
  const [mode, setMode] = useState(LIGHTING_MODES[0]); // Stores the selected mode
  const [pattern, setPattern] = useState(LIGHTING_PATTERNS[0]); // Stores the selected pattern
  const [interval, setInterval] = useState(1000); // Interval for dynamic patterns (in milliseconds)

  /*
  # Function: handleColorChange
  Updates the color state when a new color is selected in the color picker.
  */
  const handleColorChange = (color: any) => {
    setColor(color.hex);
  };

  /*
  # Function: handleModeChange
  Updates the selected mode state based on user selection from the dropdown.
  */
  const handleModeChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setMode(event.target.value);
  };

  /*
  # Function: handlePatternChange
  Updates the selected pattern state based on user selection from the dropdown.
  */
  const handlePatternChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setPattern(event.target.value);
  };

  /*
  # Function: handleIntervalChange
  Validates and updates the interval state for dynamic LED patterns.
  */
  const handleIntervalChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);
    if (value > 0) {
      setInterval(value);
    } else {
      console.warn('Interval must be a positive number.');
    }
  };

  /*
  # Function: handleApply
  Validates and sends the selected LED settings to the backend.
  */
  const handleApply = () => {
    if (pattern !== 'static' && interval <= 0) {
      console.error('Interval must be greater than 0 for dynamic patterns.');
      return;
    }

    const commandData = {
      color,
      mode,
      pattern,
      interval: pattern !== 'static' ? interval : undefined, // Include interval only for non-static patterns
    };

    sendCommand(COMMAND.SET_LED, commandData); // Send the command
    console.log('LED settings applied:', commandData); // Log the applied settings
  };

  return (
    <div className="bg-gray-900 p-4 rounded-lg shadow-md w-full md:w-3/4 lg:w-1/2">
      {/* Title */}
      <h2 className="text-lg font-bold text-green-400 border-b-2 border-green-400 pb-2">
        LED Control
      </h2>

      <div className="mt-4">
        {/* Color Picker */}
        <SketchPicker color={color} onChange={handleColorChange} />

        <div className="mt-4">
          {/* Mode Dropdown */}
          <label className="block text-green-300 mb-1">Mode:</label>
          <select
            value={mode}
            onChange={handleModeChange}
            className="w-full p-2 rounded bg-gray-800 text-green-300"
          >
            {LIGHTING_MODES.map((modeOption) => (
              <option key={modeOption} value={modeOption}>
                {modeOption.charAt(0).toUpperCase() + modeOption.slice(1)} {/* Capitalize first letter */}
              </option>
            ))}
          </select>
        </div>

        <div className="mt-4">
          {/* Pattern Dropdown */}
          <label className="block text-green-300 mb-1">Pattern:</label>
          <select
            value={pattern}
            onChange={handlePatternChange}
            className="w-full p-2 rounded bg-gray-800 text-green-300"
          >
            {LIGHTING_PATTERNS.map((patternOption) => (
              <option key={patternOption} value={patternOption}>
                {patternOption.charAt(0).toUpperCase() + patternOption.slice(1)}
              </option>
            ))}
          </select>
        </div>

        {pattern !== 'static' && (
          <div className="mt-4">
            {/* Interval Input */}
            <label className="block text-green-300 mb-1">Interval (ms):</label>
            <input
              type="number"
              value={interval}
              onChange={handleIntervalChange}
              className="w-full p-2 rounded bg-gray-800 text-green-300"
              min={1}
              placeholder="Enter interval in ms"
            />
          </div>
        )}

        {/* Apply Button */}
        <button
          onClick={handleApply}
          className="mt-4 w-full bg-green-500 p-2 rounded text-white hover:bg-green-600"
        >
          Apply
        </button>
      </div>
    </div>
  );
};

export default LedControl;
