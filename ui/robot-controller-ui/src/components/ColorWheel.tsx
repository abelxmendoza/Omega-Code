// File: /Omega-Code/ui/robot-controller-ui/components/ColorWheel.tsx

/*
This component provides a color picker using the HexColorPicker component from the react-colorful library.
Users can select a color, and the selected color is passed to the parent component via a callback function.
It also includes a button to confirm the selected color.
*/

import React, { useState } from 'react';
import { HexColorPicker } from 'react-colorful';

const ColorWheel: React.FC<{ onSelectColor: (color: string) => void }> = ({ onSelectColor }) => {
  // State to store the selected color
  const [color, setColor] = useState("#aabbcc");

  // Handle color change from the color picker
  const handleChange = (newColor: string) => {
    setColor(newColor);
    onSelectColor(newColor); // Update the parent component with the new color
  };

  return (
    <div>
      {/* HexColorPicker component for selecting color */}
      <HexColorPicker color={color} onChange={handleChange} />
      <div className="mt-2">
        {/* Button to confirm the selected color */}
        <button 
          className="bg-blue-500 text-white p-2 rounded" 
          onClick={() => onSelectColor(color)}
        >
          Select Color
        </button>
      </div>
    </div>
  );
};

export default ColorWheel;
