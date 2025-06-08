/*
# File: /src/components/lighting/LedModal.tsx
# Summary:
Provides a modal interface for controlling LED settings on the robot. Users can configure color, mode, pattern, and interval settings, and send these configurations to the backend via WebSocket.
*/

import React, { useState, useEffect, useRef } from 'react';
import { SketchPicker } from 'react-color';
import { COMMAND, LIGHTING_MODES, LIGHTING_PATTERNS } from '../../control_definitions';

interface LedModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const LedModal: React.FC<LedModalProps> = ({ isOpen, onClose }) => {
  const [color, setColor] = useState('#ffffff'); // Selected color state
  const [mode, setMode] = useState(LIGHTING_MODES[0]); // Selected mode state
  const [pattern, setPattern] = useState(LIGHTING_PATTERNS[0]); // Selected pattern state
  const [interval, setInterval] = useState(1000); // Interval for dynamic patterns (ms)
  const ws = useRef<WebSocket | null>(null);
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://localhost:8080/ws'; // WebSocket URL

  // Initialize WebSocket connection
  useEffect(() => {
    ws.current = new WebSocket(wsUrl);

    ws.current.onopen = () => console.log('WebSocket connection established');
    ws.current.onclose = () => console.log('WebSocket connection closed');
    ws.current.onerror = (error) => console.error('WebSocket error:', error);

    return () => ws.current?.close(); // Clean up WebSocket connection
  }, [wsUrl]);

  // Handle color picker changes
  const handleColorChange = (color: any) => {
    setColor(color.hex);
  };

  // Handle mode dropdown changes
  const handleModeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setMode(e.target.value);
  };

  // Handle pattern dropdown changes
  const handlePatternChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setPattern(e.target.value);
  };

  // Handle interval input changes
  const handleIntervalChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    if (value >= 100) {
      setInterval(value);
    } else {
      console.warn('Interval must be at least 100 ms.');
    }
  };

  // Send LED settings to the backend
  const handleApply = () => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      const commandData = {
        command: COMMAND.SET_LED,
        color,
        mode,
        pattern,
        interval: pattern !== 'static' ? interval : undefined, // Include interval only for dynamic patterns
      };
      ws.current.send(JSON.stringify(commandData));
      console.log('LED settings applied:', commandData);
    } else {
      console.error('WebSocket connection is not open.');
    }
  };

  // Render nothing if the modal is not open
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-gray-800 bg-opacity-75 flex justify-center items-center z-50">
      <div className="bg-gray-900 rounded-lg p-6 w-full max-w-lg relative text-white">
        {/* Close Button */}
        <button
          className="absolute top-3 right-3 text-white bg-red-500 hover:bg-red-600 p-2 rounded"
          onClick={onClose}
        >
          Close
        </button>
        {/* Title */}
        <h2 className="text-lg font-bold text-green-400 border-b-2 border-green-400 pb-2 mb-4">
          LED Configuration
        </h2>
        {/* Color Picker */}
        <SketchPicker color={color} onChange={handleColorChange} />
        {/* Mode Selector */}
        <div className="mt-4">
          <label htmlFor="mode" className="block text-green-300 font-semibold">Mode:</label>
          <select
            id="mode"
            value={mode}
            onChange={handleModeChange}
            className="w-full bg-gray-800 text-white p-2 rounded mt-1"
          >
            {LIGHTING_MODES.map((modeOption) => (
              <option key={modeOption} value={modeOption}>
                {modeOption.charAt(0).toUpperCase() + modeOption.slice(1)}
              </option>
            ))}
          </select>
        </div>
        {/* Pattern Selector */}
        <div className="mt-4">
          <label htmlFor="pattern" className="block text-green-300 font-semibold">Pattern:</label>
          <select
            id="pattern"
            value={pattern}
            onChange={handlePatternChange}
            className="w-full bg-gray-800 text-white p-2 rounded mt-1"
          >
            {LIGHTING_PATTERNS.map((patternOption) => (
              <option key={patternOption} value={patternOption}>
                {patternOption.charAt(0).toUpperCase() + patternOption.slice(1)}
              </option>
            ))}
          </select>
        </div>
        {/* Interval Input (Only for Non-Static Patterns) */}
        {pattern !== 'static' && (
          <div className="mt-4">
            <label htmlFor="interval" className="block text-green-300 font-semibold">Interval (ms):</label>
            <input
              id="interval"
              type="number"
              value={interval}
              onChange={handleIntervalChange}
              className="w-full bg-gray-800 text-white p-2 rounded mt-1"
              min={100}
            />
          </div>
        )}
        {/* Apply Button */}
        <button
          onClick={handleApply}
          className="bg-blue-500 hover:bg-blue-600 text-white p-3 rounded mt-6 w-full"
        >
          Apply Settings
        </button>
      </div>
    </div>
  );
};

export default LedModal;
