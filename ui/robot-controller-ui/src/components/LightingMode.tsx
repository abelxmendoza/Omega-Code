import React from 'react';

interface LightingModeProps {
  mode?: 'single' | 'rainbow';
  onModeChange?: (mode: string) => void;
  onSelectMode?: (mode: string) => void; // For test compatibility
}

const LightingMode: React.FC<LightingModeProps> = ({ 
  mode = 'single', 
  onModeChange,
  onSelectMode
}) => {
  const handleChange = (value: string) => {
    onModeChange?.(value);
    onSelectMode?.(value); // Support both prop names
  };

  return (
    <div>
      <h3>Lighting Mode</h3>
      <div>
        <button onClick={() => handleChange('single')}>Single Color</button>
        <button onClick={() => handleChange('rainbow')}>Rainbow</button>
      </div>
      <select value={mode} onChange={(e) => handleChange(e.target.value)}>
        <option value="single">Single</option>
        <option value="rainbow">Rainbow</option>
      </select>
    </div>
  );
};

export default LightingMode;
