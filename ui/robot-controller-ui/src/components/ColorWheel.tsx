import React from 'react';

interface ColorWheelProps {
  color?: string;
  onColorChange?: (color: string) => void;
}

const ColorWheel: React.FC<ColorWheelProps> = ({ 
  color = '#ff0000', 
  onColorChange 
}) => {
  return (
    <div>
      <h3>Color Wheel</h3>
      <input
        type="color"
        value={color}
        onChange={(e) => onColorChange?.(e.target.value)}
      />
    </div>
  );
};

export default ColorWheel;
