// File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LightingPattern.tsx

/*
# Summary:
Provides a selection interface for various lighting patterns.
Users can choose from "Static," "Blink," "Fade," "Chase," and "Rainbow" patterns.
The selected pattern is passed to the parent component via the `onSelectPattern` callback.
*/

import React, { useState } from 'react';

// Props interface for the LightingPattern component
interface LightingPatternProps {
  onSelectPattern: (pattern: string) => void; // Callback function to notify the parent component of the selected pattern
}

const LightingPattern: React.FC<LightingPatternProps> = ({ onSelectPattern }) => {
  // State to keep track of the selected pattern
  const [selectedPattern, setSelectedPattern] = useState('static');

  /**
   * Handles pattern selection.
   * Updates the selected pattern state and notifies the parent component.
   * 
   * @param pattern - The lighting pattern selected by the user.
   */
  const handlePatternChange = (pattern: string) => {
    setSelectedPattern(pattern);
    onSelectPattern(pattern); // Notify the parent component of the pattern change
  };

  // List of available patterns
  const patterns = ['static', 'blink', 'fade', 'chase', 'rainbow'];

  return (
    <div className="flex flex-wrap justify-around items-center bg-gray-900 p-4 rounded-lg shadow-md">
      {/* Render pattern selection buttons */}
      {patterns.map((pattern) => (
        <button
          key={pattern}
          className={`p-3 m-2 rounded transition ${
            selectedPattern === pattern ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
          } hover:bg-blue-700`}
          onClick={() => handlePatternChange(pattern)}
        >
          {pattern.charAt(0).toUpperCase() + pattern.slice(1)} {/* Capitalize first letter */}
        </button>
      ))}
    </div>
  );
};

export default LightingPattern;
