import React from 'react';

interface LightingPatternProps {
  pattern?: 'static' | 'pulse' | 'blink' | 'music';
  onPatternChange?: (pattern: string) => void;
}

const LightingPattern: React.FC<LightingPatternProps> = ({ 
  pattern = 'static', 
  onPatternChange 
}) => {
  return (
    <div>
      <h3>Lighting Pattern</h3>
      <select value={pattern} onChange={(e) => onPatternChange?.(e.target.value)}>
        <option value="static">Static</option>
        <option value="pulse">Pulse</option>
        <option value="blink">Blink</option>
        <option value="music">Music</option>
      </select>
    </div>
  );
};

export default LightingPattern;
