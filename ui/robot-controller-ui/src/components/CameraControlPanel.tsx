import React from 'react';

interface CameraControlPanelProps {
  onCommand?: (command: string) => void;
  tilt?: number;
  pan?: number;
  onTiltChange?: (tilt: number) => void;
  onPanChange?: (pan: number) => void;
}

const CameraControlPanel: React.FC<CameraControlPanelProps> = ({ 
  onCommand,
  tilt = 0,
  pan = 0,
  onTiltChange,
  onPanChange
}) => {
  return (
    <div className="p-4 bg-gray-800 rounded-lg">
      <h3 className="text-white font-bold mb-4">Camera Control Panel</h3>
      
      <div className="space-y-4">
        {/* Tilt Control */}
        <div>
          <label className="block text-white text-sm mb-2">Tilt: {tilt}°</label>
          <input
            type="range"
            min="-45"
            max="45"
            value={tilt}
            onChange={(e) => onTiltChange?.(Number(e.target.value))}
            className="w-full"
          />
        </div>

        {/* Pan Control */}
        <div>
          <label className="block text-white text-sm mb-2">Pan: {pan}°</label>
          <input
            type="range"
            min="-90"
            max="90"
            value={pan}
            onChange={(e) => onPanChange?.(Number(e.target.value))}
            className="w-full"
          />
        </div>

        {/* Movement Controls */}
        <div className="grid grid-cols-2 gap-2">
          <button 
            className="p-3 bg-blue-600 hover:bg-blue-700 text-white rounded font-semibold"
            onClick={() => onCommand?.('up')}
          >
            ↑ Up
          </button>
          <button 
            className="p-3 bg-blue-600 hover:bg-blue-700 text-white rounded font-semibold"
            onClick={() => onCommand?.('down')}
          >
            ↓ Down
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

        {/* Reset Button */}
        <button 
          className="w-full p-3 bg-gray-600 hover:bg-gray-700 text-white rounded font-semibold"
          onClick={() => onCommand?.('reset')}
        >
          🔄 Reset Position
        </button>
      </div>
    </div>
  );
};

export default CameraControlPanel;
