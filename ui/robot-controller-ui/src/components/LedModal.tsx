/*
# File: /src/components/LedModal.tsx
# Summary:
Provides a modal interface for controlling LED settings on the robot.
*/

import React, { useState, useEffect, useRef } from 'react';
import { SketchPicker } from 'react-color';

const LedModal: React.FC<{ isOpen: boolean; onClose: () => void }> = ({ isOpen, onClose }) => {
  const [color, setColor] = useState('#ffffff');
  const [mode, setMode] = useState('single');
  const [pattern, setPattern] = useState('static');
  const [interval, setInterval] = useState(1000);
  const ws = useRef<WebSocket | null>(null);
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://localhost:8080/ws';

  useEffect(() => {
    ws.current = new WebSocket(wsUrl);

    ws.current.onopen = () => console.log('WebSocket connection established');
    ws.current.onclose = () => console.log('WebSocket connection closed');
    ws.current.onerror = (error) => console.error('WebSocket error:', error);

    return () => ws.current?.close();
  }, [wsUrl]);

  const handleColorChange = (color: any) => setColor(color.hex);

  const handleModeChange = (e: React.ChangeEvent<HTMLSelectElement>) => setMode(e.target.value);

  const handlePatternChange = (e: React.ChangeEvent<HTMLSelectElement>) => setPattern(e.target.value);

  const handleIntervalChange = (e: React.ChangeEvent<HTMLInputElement>) => setInterval(Number(e.target.value));

  const handleApply = () => {
    if (ws.current) {
      ws.current.send(
        JSON.stringify({
          command: 'set-led',
          color,
          mode,
          pattern,
          interval: pattern !== 'static' ? interval : undefined,
        })
      );
    }
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-gray-800 bg-opacity-75 flex justify-center items-center z-50">
      <div className="bg-gray-900 rounded-lg p-4 w-full max-w-lg relative text-white">
        <button className="absolute top-0 right-0 m-4 text-white" onClick={onClose}>
          X
        </button>
        <p className="text-sm text-gray-400 font-bold mb-4">
          ⚠️ Double-click the &quot;I&quot; key to turn the lights on.
        </p>
        <SketchPicker color={color} onChange={handleColorChange} />
        <div>
          <label htmlFor="mode" className="block mt-4">Mode:</label>
          <select id="mode" value={mode} onChange={handleModeChange} className="bg-gray-800 text-white rounded p-1">
            <option value="single">Single Color</option>
            <option value="multi">Multi Color</option>
            <option value="two">Two Color</option>
          </select>
        </div>
        <div>
          <label htmlFor="pattern" className="block mt-4">Pattern:</label>
          <select id="pattern" value={pattern} onChange={handlePatternChange} className="bg-gray-800 text-white rounded p-1">
            <option value="static">Static</option>
            <option value="blink">Blink</option>
            <option value="fade">Fade</option>
          </select>
        </div>
        {pattern !== 'static' && (
          <div>
            <label htmlFor="interval" className="block mt-4">Interval (ms):</label>
            <input
              id="interval"
              type="number"
              value={interval}
              onChange={handleIntervalChange}
              className="bg-gray-800 text-white rounded p-1"
            />
          </div>
        )}
        <button onClick={handleApply} className="bg-blue-500 text-white p-2 rounded mt-4">
          Apply
        </button>
      </div>
    </div>
  );
};

export default LedModal;
