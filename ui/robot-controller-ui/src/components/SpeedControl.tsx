import React from 'react';

interface SpeedControlProps {
  speed?: number;
  onSpeedChange?: (speed: number) => void;
}

const SpeedControl: React.FC<SpeedControlProps> = ({ 
  speed = 50, 
  onSpeedChange 
}) => {
  return (
    <div>
      <h3>Speed Control</h3>
      <input
        type="range"
        min="0"
        max="100"
        value={speed}
        onChange={(e) => onSpeedChange?.(Number(e.target.value))}
      />
      <span>{speed}%</span>
    </div>
  );
};

export default SpeedControl;
