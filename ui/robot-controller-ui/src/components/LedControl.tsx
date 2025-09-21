import React from 'react';

interface LedControlProps {
  color?: string;
  brightness?: number;
  onColorChange?: (color: string) => void;
  onBrightnessChange?: (brightness: number) => void;
}

const LedControl: React.FC<LedControlProps> = ({ 
  color = '#ff0000', 
  brightness = 50,
  onColorChange,
  onBrightnessChange
}) => {
  return (
    <div>
      <h3>LED Control</h3>
      <div>
        <label>Color:</label>
        <input
          type="color"
          value={color}
          onChange={(e) => onColorChange?.(e.target.value)}
        />
      </div>
      <div>
        <label>Brightness:</label>
        <input
          type="range"
          min="0"
          max="100"
          value={brightness}
          onChange={(e) => onBrightnessChange?.(Number(e.target.value))}
        />
      </div>
    </div>
  );
};

export default LedControl;
