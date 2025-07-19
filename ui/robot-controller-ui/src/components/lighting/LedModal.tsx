/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LedModal.tsx
# Summary:
Modal interface for controlling LED lighting on the robot. Supports single and two-color modes,
pattern selection, brightness control, and interval configuration for dynamic patterns.
Sends configuration to the backend via WebSocket.
*/

import React, { useState, useEffect, useRef } from 'react';
import { SketchPicker } from 'react-color';
import { COMMAND, LIGHTING_MODES, LIGHTING_PATTERNS } from '../../control_definitions';

interface LedModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const LedModal: React.FC<LedModalProps> = ({ isOpen, onClose }) => {
  const [color1, setColor1] = useState('#ffffff');
  const [color2, setColor2] = useState('#000000');
  const [mode, setMode] = useState(LIGHTING_MODES[0]);
  const [pattern, setPattern] = useState(LIGHTING_PATTERNS[0]);
  const [interval, setInterval] = useState(1000);
  const [brightness, setBrightness] = useState(100); // 0–100%
  const ws = useRef<WebSocket | null>(null);
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://localhost:8080/ws';

  useEffect(() => {
    ws.current = new WebSocket(wsUrl);
    ws.current.onopen = () => console.log('WebSocket connection established');
    ws.current.onclose = () => console.log('WebSocket connection closed');
    ws.current.onerror = (error) => console.error('WebSocket error:', error);
    return () => ws.current?.close();
  }, [wsUrl]);

  const handleColor1Change = (color: any) => setColor1(color.hex);
  const handleColor2Change = (color: any) => setColor2(color.hex);
  const handleModeChange = (e: React.ChangeEvent<HTMLSelectElement>) => setMode(e.target.value);
  const handlePatternChange = (e: React.ChangeEvent<HTMLSelectElement>) => setPattern(e.target.value);
  const handleIntervalChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    if (value >= 100) setInterval(value);
    else console.warn('Interval must be at least 100 ms.');
  };
  const handleBrightnessChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    if (value >= 0 && value <= 100) setBrightness(value);
  };

  const handleApply = () => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      const commandData: any = {
        command: COMMAND.SET_LED,
        mode,
        pattern,
        color1,
        brightness: brightness / 100,
      };

      if (mode === 'two') {
        commandData.color2 = color2;
      }

      if (pattern !== 'static') {
        commandData.interval = interval;
      }

      ws.current.send(JSON.stringify(commandData));
      console.log('LED settings applied:', commandData);
    } else {
      console.error('WebSocket connection is not open.');
    }
  };

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

        {/* Color Picker 1 */}
        <label className="block text-green-300 font-semibold mb-1">Primary Color:</label>
        <SketchPicker color={color1} onChange={handleColor1Change} />

        {/* Color Picker 2 */}
        {mode === 'two' && (
          <>
            <label className="block text-green-300 font-semibold mt-4 mb-1">Secondary Color:</label>
            <SketchPicker color={color2} onChange={handleColor2Change} />
          </>
        )}

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

        {/* Interval Input */}
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

        {/* Brightness Slider */}
        <div className="mt-4">
          <label htmlFor="brightness" className="block text-green-300 font-semibold">Brightness (%):</label>
          <input
            id="brightness"
            type="range"
            min={0}
            max={100}
            value={brightness}
            onChange={handleBrightnessChange}
            className="w-full"
          />
        </div>

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
