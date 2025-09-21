import React from 'react';

interface LedControlProps {
  color?: string;
  brightness?: number;
  mode?: 'single' | 'multi';
  pattern?: 'static' | 'pulse' | 'blink' | 'music';
  interval?: number;
  onColorChange?: (color: string) => void;
  onBrightnessChange?: (brightness: number) => void;
  onModeChange?: (mode: string) => void;
  onPatternChange?: (pattern: string) => void;
  onIntervalChange?: (interval: number) => void;
}

const LedControl: React.FC<LedControlProps> = ({ 
  color = '#ff0000', 
  brightness = 50,
  mode = 'single',
  pattern = 'static',
  interval = 1000,
  onColorChange,
  onBrightnessChange,
  onModeChange,
  onPatternChange,
  onIntervalChange
}) => {
  return (
    <div className="p-4 bg-gray-800 rounded-lg">
      <h3 className="text-white font-bold mb-4">LED Control</h3>
      
      <div className="space-y-4">
        <div>
          <label className="block text-white text-sm mb-2">Mode:</label>
          <select 
            value={mode} 
            onChange={(e) => onModeChange?.(e.target.value)}
            className="w-full p-2 bg-gray-700 text-white rounded"
          >
            <option value="single">Single</option>
            <option value="multi">Multi</option>
          </select>
        </div>

        <div>
          <label className="block text-white text-sm mb-2">Pattern:</label>
          <select 
            value={pattern} 
            onChange={(e) => onPatternChange?.(e.target.value)}
            className="w-full p-2 bg-gray-700 text-white rounded"
          >
            <option value="static">Static</option>
            <option value="pulse">Pulse</option>
            <option value="blink">Blink</option>
            <option value="music">Music</option>
          </select>
        </div>

        <div>
          <label className="block text-white text-sm mb-2">Color:</label>
          <input
            type="color"
            value={color}
            onChange={(e) => onColorChange?.(e.target.value)}
            className="w-full h-10 bg-gray-700 rounded"
          />
        </div>

        <div>
          <label className="block text-white text-sm mb-2">Brightness: {brightness}%</label>
          <input
            type="range"
            min="0"
            max="100"
            value={brightness}
            onChange={(e) => onBrightnessChange?.(Number(e.target.value))}
            className="w-full"
          />
        </div>

        <div>
          <label className="block text-white text-sm mb-2">Interval (ms):</label>
          <input
            type="number"
            min="100"
            max="5000"
            value={interval}
            onChange={(e) => onIntervalChange?.(Number(e.target.value))}
            className="w-full p-2 bg-gray-700 text-white rounded"
          />
        </div>
      </div>
    </div>
  );
};

export default LedControl;
