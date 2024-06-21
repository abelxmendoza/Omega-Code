// File: /Omega-Code/ui/robot-controller-ui/src/components/LedModal.tsx
import React, { useState } from 'react';
import { SketchPicker } from 'react-color';

const LedModal: React.FC<{ sendCommand: (command: string) => void, isOpen: boolean, onClose: () => void }> = ({ sendCommand, isOpen, onClose }) => {
  const [color, setColor] = useState('#ffffff');
  const [mode, setMode] = useState('single');
  const [pattern, setPattern] = useState('static');
  const [interval, setInterval] = useState(1000);

  const handleColorChange = (color: any) => {
    setColor(color.hex);
  };

  const handleModeChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setMode(event.target.value);
  };

  const handlePatternChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setPattern(event.target.value);
  };

  const handleIntervalChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInterval(Number(event.target.value));
  };

  const handleApply = () => {
    sendCommand(JSON.stringify({
      command: 'set-led',
      color,
      mode,
      pattern,
      interval: pattern !== 'static' ? interval : undefined,
    }));
  };

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-gray-800 bg-opacity-75 flex justify-center items-center z-50">
      <div className="bg-gray-900 rounded-lg p-4 w-full max-w-lg relative text-white">
        <button className="absolute top-0 right-0 m-4 text-white" onClick={onClose}>X</button>
        <SketchPicker color={color} onChange={handleColorChange} />
        <div>
          <label className="block mt-4">Mode:</label>
          <select value={mode} onChange={handleModeChange} className="bg-gray-800 text-white rounded p-1">
            <option value="single">Single Color</option>
            <option value="multi">Multi Color</option>
            <option value="two">Two Color</option>
          </select>
        </div>
        <div>
          <label className="block mt-4">Pattern:</label>
          <select value={pattern} onChange={handlePatternChange} className="bg-gray-800 text-white rounded p-1">
            <option value="static">Static</option>
            <option value="blink">Blink</option>
            <option value="fade">Fade</option>
          </select>
        </div>
        {pattern !== 'static' && (
          <div>
            <label className="block mt-4">Interval (ms):</label>
            <input type="number" value={interval} onChange={handleIntervalChange} className="bg-gray-800 text-white rounded p-1" />
          </div>
        )}
        <button onClick={handleApply} className="bg-blue-500 text-white p-2 rounded mt-4">Apply</button>
      </div>
    </div>
  );
};

export default LedModal;
