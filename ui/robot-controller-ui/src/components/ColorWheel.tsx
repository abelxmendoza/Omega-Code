// File: /Omega-Code/ui/robot-controller-ui/components/ColorWheel.tsx
import React, { useState } from 'react';
import { HexColorPicker } from "react-colorful";

const ColorWheel: React.FC<{ onSelectColor: (color: string) => void }> = ({ onSelectColor }) => {
  const [color, setColor] = useState("#aabbcc");

  const handleChange = (newColor: string) => {
    setColor(newColor);
    onSelectColor(newColor);
  };

  return (
    <div>
      <HexColorPicker color={color} onChange={handleChange} />
      <div className="mt-2">
        <button className="bg-blue-500 text-white p-2 rounded" onClick={() => onSelectColor(color)}>
          Select Color
        </button>
      </div>
    </div>
  );
};

export default ColorWheel;

