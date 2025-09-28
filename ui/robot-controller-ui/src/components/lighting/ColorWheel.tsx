// File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/ColorWheel.tsx

/*
This component provides a simple color picker using HTML input elements.
Users can select a color, and the selected color is passed to the parent component via a callback function.
It also includes a button to confirm the selected color.
*/

import React, { useState } from 'react';

const ColorWheel: React.FC<{ onSelectColor: (color: string) => void }> = ({ onSelectColor }) => {
  // State to store the selected color
  const [color, setColor] = useState("#aabbcc");

  // Handle color change from the color picker
  const handleChange = (newColor: string) => {
    setColor(newColor);
  };

  // Confirm the selected color and pass it to the parent component
  const handleConfirmColor = () => {
    onSelectColor(color);
    console.log("Color selected:", color);
  };

  return (
    <div className="p-4 bg-gray-900 rounded-lg shadow-md text-center">
      <h2 className="text-lg font-bold text-green-400 mb-4">Color Picker</h2>
      {/* Simple HTML color input for selecting color */}
      <div className="mb-4 flex justify-center">
        <input
          type="color"
          value={color}
          onChange={(e) => handleChange(e.target.value)}
          className="w-32 h-32 rounded-lg border-2 border-gray-600 cursor-pointer"
        />
      </div>
      <div className="mt-2">
        {/* Display the currently selected color */}
        <div className="text-green-300 font-semibold mb-2">
          Selected Color: <span style={{ color }}>{color}</span>
        </div>
        {/* Button to confirm the selected color */}
        <button
          className="bg-blue-500 hover:bg-blue-600 text-white py-2 px-4 rounded"
          onClick={handleConfirmColor}
        >
          Confirm Color
        </button>
      </div>
    </div>
  );
};

export default ColorWheel;

