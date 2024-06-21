// File: /Omega-Code/ui/robot-controller-ui/src/components/LedControl.tsx
import React, { useState } from 'react';
import { SketchPicker } from 'react-color';

const LedControl: React.FC<{ sendCommand: (command: string) => void }> = ({ sendCommand }) => {
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
      interval,
    }));
  };

  return (
    <div className="bg-gray-900 p-4 rounded-lg shadow-md w-full md:w-3/4 lg:w-1/2">
      <h2 className="text-lg font-bold text-green-400 border-b-2 border-green-400 pb-2">LED Control</h2>
      <div className="mt-4">
        <SketchPicker color={color} onChange={handleColorChange} />
        <div className="mt-4">
          <label className="block text-green-300">Mode:</label>
          <select value={mode} onChange={handleModeChange} className="w-full p-2 rounded bg-gray-800 text-green-300">
            <option value="single">Single Color</option>
            <option value="multi">Multi Color</option>
            <option value="two">Two Color</option>
          </select>
        </div>
        <div className="mt-4">
          <label className="block text-green-300">Pattern:</label>
          <select value={pattern} onChange={handlePatternChange} className="w-full p-2 rounded bg-gray-800 text-green-300">
            <option value="static">Static</option>
            <option value="blink">Blink</option>
            <option value="fade">Fade</option>
          </select>
        </div>
        <div className="mt-4">
          <label className="block text-green-300">Interval (ms):</label>
          <input type="number" value={interval} onChange={handleIntervalChange} className="w-full p-2 rounded bg-gray-800 text-green-300" />
        </div>
        <button onClick={handleApply} className="mt-4 w-full bg-green-500 p-2 rounded text-white">Apply</button>
      </div>
    </div>
  );
};

export default LedControl;
