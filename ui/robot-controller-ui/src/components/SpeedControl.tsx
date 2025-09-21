import React from 'react';

interface SpeedControlProps {
  speed?: number;
  onSpeedChange?: (speed: number) => void;
  label?: string;
  min?: number;
  max?: number;
}

const SpeedControl: React.FC<SpeedControlProps> = ({ 
  speed = 50, 
  onSpeedChange,
  label = "Speed",
  min = 0,
  max = 100
}) => {
  return (
    <div className="p-4 bg-gray-800 rounded-lg">
      <h3 className="text-white font-bold mb-4">{label} Control</h3>
      
      <div className="space-y-4">
        <div>
          <label className="block text-white text-sm mb-2">{label}: {speed}%</label>
          <input
            type="range"
            min={min}
            max={max}
            value={speed}
            onChange={(e) => onSpeedChange?.(Number(e.target.value))}
            className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
          />
        </div>

        {/* Quick Speed Buttons */}
        <div className="grid grid-cols-4 gap-2">
          <button 
            className="p-2 bg-gray-600 hover:bg-gray-700 text-white text-sm rounded"
            onClick={() => onSpeedChange?.(25)}
          >
            25%
          </button>
          <button 
            className="p-2 bg-gray-600 hover:bg-gray-700 text-white text-sm rounded"
            onClick={() => onSpeedChange?.(50)}
          >
            50%
          </button>
          <button 
            className="p-2 bg-gray-600 hover:bg-gray-700 text-white text-sm rounded"
            onClick={() => onSpeedChange?.(75)}
          >
            75%
          </button>
          <button 
            className="p-2 bg-gray-600 hover:bg-gray-700 text-white text-sm rounded"
            onClick={() => onSpeedChange?.(100)}
          >
            100%
          </button>
        </div>
      </div>
    </div>
  );
};

export default SpeedControl;
