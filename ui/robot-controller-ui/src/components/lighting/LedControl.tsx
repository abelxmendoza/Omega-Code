/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LedControl.tsx
# Summary:
Allows the user to configure LED settings for the robot, including color, mode, pattern, interval, and brightness.
Settings are sent as commands to the backend WebSocket server.
*/

import React, { useState } from 'react';
import { SketchPicker, type ColorResult } from 'react-color';
import { COMMAND, LIGHTING_MODES, LIGHTING_PATTERNS } from '../../control_definitions';

// Derive union types from your constant arrays (falls back to string if not `as const`)
type LightingMode = (typeof LIGHTING_MODES)[number];
type LightingPattern = (typeof LIGHTING_PATTERNS)[number];

// Props definition for the component
interface LedControlProps {
  sendCommand: (command: string, data?: any) => void; // Function to send commands to the backend
}

const LedControl: React.FC<LedControlProps> = ({ sendCommand }) => {
  // Component states for managing LED settings
  const [color, setColor] = useState('#ffffff'); // Stores the selected color

  // Explicit generics so setMode/setPattern accept union (or string) instead of a single literal
  const [mode, setMode] = useState<LightingMode>((LIGHTING_MODES?.[0] ?? 'single') as LightingMode);
  const [pattern, setPattern] = useState<LightingPattern>((LIGHTING_PATTERNS?.[0] ?? 'static') as LightingPattern);

  const [interval, setInterval] = useState(1000); // Interval for dynamic patterns (in milliseconds)
  const [brightness, setBrightness] = useState(100); // Brightness in percent (0–100)

  // Color change handler
  const handleColorChange = (color: ColorResult) => setColor(color.hex);

  // Mode change handler
  const handleModeChange = (event: React.ChangeEvent<HTMLSelectElement>) =>
    setMode(event.target.value as LightingMode);

  // Pattern change handler
  const handlePatternChange = (event: React.ChangeEvent<HTMLSelectElement>) =>
    setPattern(event.target.value as LightingPattern);

  // Interval change handler
  const handleIntervalChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(event.target.value);
    if (value > 0) setInterval(value);
    else console.warn('Interval must be a positive number.');
  };

  // Brightness change handler
  const handleBrightnessChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    let val = Number(event.target.value);
    if (val > 100) val = 100;
    if (val < 0) val = 0;
    setBrightness(val);
  };

  // Convert hex color to 24-bit int for backend compatibility
  function hexToInt(hex: string): number {
    return parseInt(hex.replace(/^#/, ''), 16);
  }

  // Send all LED settings to backend
  const handleApply = () => {
    if (pattern !== 'static' && interval <= 0) {
      console.error('Interval must be greater than 0 for dynamic patterns.');
      return;
    }

    const commandData = {
      color: hexToInt(color),
      mode,
      pattern,
      interval: pattern !== 'static' ? interval : undefined,
      brightness: brightness / 100, // always a float between 0–1
    };

    sendCommand(COMMAND.SET_LED, commandData);
    console.log('LED settings applied:', commandData);
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

        {/* Mode Dropdown */}
        <div className="mt-4">
          <label className="block text-green-300 mb-1">Mode:</label>
          <select
            value={mode}
            onChange={handleModeChange}
            className="w-full p-2 rounded bg-gray-800 text-green-300"
          >
            {LIGHTING_MODES.map((modeOption) => (
              <option key={modeOption} value={modeOption}>
                {modeOption.charAt(0).toUpperCase() + modeOption.slice(1)}
              </option>
            ))}
          </select>
        </div>

        {/* Pattern Dropdown */}
        <div className="mt-4">
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

        {/* Interval Input (only for dynamic patterns) */}
        {pattern !== 'static' && (
          <div className="mt-4">
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

        {/* Brightness Slider */}
        <div className="mt-4">
          <label className="block text-green-300 mb-1">Brightness: {brightness}%</label>
          <input
            type="range"
            min={0}
            max={100}
            value={brightness}
            onChange={handleBrightnessChange}
            className="w-full accent-green-400"
          />
        </div>

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
