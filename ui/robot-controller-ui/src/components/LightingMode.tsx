import React from 'react';

interface LightingModeProps {
  mode?: 'single' | 'rainbow';
  onModeChange?: (mode: string) => void;
}

const LightingMode: React.FC<LightingModeProps> = ({ 
  mode = 'single', 
  onModeChange 
}) => {
  return (
    <div>
      <h3>Lighting Mode</h3>
      <select value={mode} onChange={(e) => onModeChange?.(e.target.value)}>
        <option value="single">Single</option>
        <option value="rainbow">Rainbow</option>
      </select>
    </div>
  );
};

export default LightingMode;
