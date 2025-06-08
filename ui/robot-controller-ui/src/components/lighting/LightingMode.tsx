// File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LightingMode.tsx

/*
# Summary:
Provides a selection interface for various lighting modes.
Users can choose between "Single Color," "Multi-Color," and "Two Color" modes.
The selected mode is passed to the parent component via the `onSelectMode` callback.
*/

import React, { useState } from 'react';

// Props interface for the LightingMode component
interface LightingModeProps {
  onSelectMode: (mode: string) => void; // Callback function to notify the parent component of the selected mode
}

const LightingMode: React.FC<LightingModeProps> = ({ onSelectMode }) => {
  // State to keep track of the selected mode
  const [selectedMode, setSelectedMode] = useState('single');

  /**
   * Handles mode selection.
   * Updates the selected mode state and notifies the parent component.
   * 
   * @param mode - The lighting mode selected by the user.
   */
  const handleModeChange = (mode: string) => {
    setSelectedMode(mode);
    onSelectMode(mode); // Notify the parent component of the mode change
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

      {/* Multi-Color Button */}
      <button
        className={`p-3 rounded transition ${
          selectedMode === 'multi' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
        } hover:bg-blue-700`}
        onClick={() => handleModeChange('multi')}
      >
        Multi-Color
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
