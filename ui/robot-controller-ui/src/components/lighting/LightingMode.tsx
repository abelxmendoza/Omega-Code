/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LightingMode.tsx
# Summary:
Provides a selection interface for lighting modes.
Users can choose between "Single Color" and "Two Color" modes.
The selected mode is passed to the parent component via the `onSelectMode` callback.
*/

import React, { useState } from 'react';

// Props interface for the LightingMode component
interface LightingModeProps {
  onSelectMode: (mode: string) => void;
}

const LightingMode: React.FC<LightingModeProps> = ({ onSelectMode }) => {
  const [selectedMode, setSelectedMode] = useState('single');

  const handleModeChange = (mode: string) => {
    setSelectedMode(mode);
    onSelectMode(mode);
  };

  return (
    <div className="flex justify-around items-center space-x-4 bg-gray-900 p-4 rounded-lg shadow-md">
      {/* Single Color Button */}
      <button
        className={`p-3 rounded transition ${
          selectedMode === 'single' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
        } hover:bg-blue-700`}
        onClick={() => handleModeChange('single')}
      >
        Single Color
      </button>

      {/* Two Color Button */}
      <button
        className={`p-3 rounded transition ${
          selectedMode === 'two' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
        } hover:bg-blue-700`}
        onClick={() => handleModeChange('two')}
      >
        Two Color
      </button>
    </div>
  );
};

export default LightingMode;
