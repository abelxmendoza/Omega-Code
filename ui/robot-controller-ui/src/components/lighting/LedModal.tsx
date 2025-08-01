/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LedModal.tsx
# Summary:
Modal interface for controlling LED lighting on the robot. Supports single and rainbow modes,
power toggle, pattern selection, brightness control, and interval configuration for dynamic patterns.
Sends configuration to the backend via WebSocket.
*/

import React, { useState, useEffect, useRef } from 'react';
import { SketchPicker } from 'react-color';
import { COMMAND } from '../../control_definitions';
import { connectLightingWs } from '../../utils/connectLightingWs';

const LIGHTING_MODES = ['single', 'rainbow'];
const LIGHTING_PATTERNS = ['static', 'pulse', 'blink'];

interface LedModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const LedModal: React.FC<LedModalProps> = ({ isOpen, onClose }) => {
  const [ledOn, setLedOn] = useState(true);
  const [color1, setColor1] = useState('#ffffff');
  const [mode, setMode] = useState(LIGHTING_MODES[0]);
  const [pattern, setPattern] = useState(LIGHTING_PATTERNS[0]);
  const [interval, setInterval] = useState(1000);
  const [brightness, setBrightness] = useState(100); // 0–100%
  const ws = useRef<WebSocket | null>(null);

  // Connect WS only when modal is open
  useEffect(() => {
    if (!isOpen) return;

    let wsInstance: WebSocket | null = null;
    connectLightingWs().then((wsObj) => {
      ws.current = wsObj;
      wsInstance = wsObj;
      wsObj.onopen = () => console.log('Lighting WS connection established');
      wsObj.onclose = () => console.log('Lighting WS connection closed');
      wsObj.onerror = (error) => console.error('Lighting WS error:', error);
    }).catch((err) => {
      console.error('Lighting WS failed to connect', err);
    });

    return () => {
      wsInstance?.close();
    };
  }, [isOpen]);

  const handleColor1Change = (color: any) => setColor1(color.hex);
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

  // ----- Updated Power Button Handler -----
  const handleTogglePower = () => {
    const newState = !ledOn;
    setLedOn(newState);

    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      if (!newState) {
        // Turn OFF: send pattern "off" (other values are ignored by backend)
        ws.current.send(
          JSON.stringify({
            color: color1,
            mode,
            pattern: "off",
            interval,
            brightness: 0,
          })
        );
        console.log("LED OFF command sent!");
      } else {
        // Turn ON: immediately apply current settings (as if user hit Apply)
        ws.current.send(
          JSON.stringify({
            color: color1,
            mode,
            pattern,
            interval,
            brightness: brightness / 100,
          })
        );
        console.log("LED ON command sent!");
      }
    } else {
      console.error('Lighting WS not open.');
    }
  };

  const handleApply = () => {
    if (!ledOn) {
      console.log('LED is off, skipping apply.');
      return;
    }
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      const commandData: any = {
        color: color1,              // Send as hex string (e.g. "#ff0000")
        mode,
        pattern,
        interval,
        brightness: brightness / 100,
      };
      ws.current.send(JSON.stringify(commandData));
      console.log('LED settings applied:', commandData);
    } else {
      console.error('Lighting WS not open.');
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

        {/* LED Power Toggle */}
        <div className="mt-2 mb-4 flex justify-between items-center">
          <span className="text-green-300 font-semibold">
            LED Status: {ledOn ? 'On' : 'Off'}
          </span>
          <button
            onClick={handleTogglePower}
            className={`px-4 py-2 rounded text-white ${ledOn ? 'bg-red-500 hover:bg-red-600' : 'bg-green-500 hover:bg-green-600'}`}
          >
            Turn {ledOn ? 'Off' : 'On'}
          </button>
        </div>

        <fieldset disabled={!ledOn} className={ledOn ? '' : 'opacity-50'}>
          {/* Color Picker 1 */}
          <label className="block text-green-300 font-semibold mb-1">Primary Color:</label>
          <SketchPicker color={color1} onChange={handleColor1Change} />

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
        </fieldset>

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
