import React from 'react';

interface CarControlPanelProps {
  onCommand?: (command: string) => void;
  speed?: number;
  onSpeedChange?: (speed: number) => void;
}

const CarControlPanel: React.FC<CarControlPanelProps> = ({ 
  onCommand,
  speed = 50,
  onSpeedChange
}) => {
  return (
    <div className="p-4 bg-gray-800 rounded-lg">
      <h3 className="text-white font-bold mb-4">Car Control Panel</h3>
      
      <div className="space-y-4">
        {/* Speed Control */}
        <div>
          <label className="block text-white text-sm mb-2">Speed: {speed}%</label>
          <input
            type="range"
            min="0"
            max="100"
            value={speed}
            onChange={(e) => onSpeedChange?.(Number(e.target.value))}
            className="w-full"
          />
        </div>

        {/* Movement Controls */}
        <div className="grid grid-cols-2 gap-2">
          <button 
            className="p-3 bg-green-600 hover:bg-green-700 text-white rounded font-semibold"
            onClick={() => onCommand?.('forward')}
          >
            ↑ Forward
          </button>
          <button 
            className="p-3 bg-red-600 hover:bg-red-700 text-white rounded font-semibold"
            onClick={() => onCommand?.('backward')}
          >
            ↓ Backward
          </button>
          <button 
            className="p-3 bg-blue-600 hover:bg-blue-700 text-white rounded font-semibold"
            onClick={() => onCommand?.('left')}
          >
            ← Left
          </button>
          <button 
            className="p-3 bg-blue-600 hover:bg-blue-700 text-white rounded font-semibold"
            onClick={() => onCommand?.('right')}
          >
            Right →
          </button>
        </div>

        {/* Stop Button */}
        <button 
          className="w-full p-3 bg-gray-600 hover:bg-gray-700 text-white rounded font-semibold"
          onClick={() => onCommand?.('stop')}
        >
          ⏹ Stop
        </button>
      </div>
    </div>
  );
};

export default CarControlPanel;
