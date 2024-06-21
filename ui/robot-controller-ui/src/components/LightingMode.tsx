// File: /Omega-Code/ui/robot-controller-ui/components/LightingMode.tsx
import React, { useState } from 'react';

const LightingMode: React.FC<{ onSelectMode: (mode: string) => void }> = ({ onSelectMode }) => {
  const [mode, setMode] = useState('single');

  const handleChange = (newMode: string) => {
    setMode(newMode);
    onSelectMode(newMode);
  };

  return (
    <div className="flex space-x-4">
      <button className={`p-2 rounded ${mode === 'single' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('single')}>Single Color</button>
      <button className={`p-2 rounded ${mode === 'multi' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('multi')}>Multi-Color</button>
      <button className={`p-2 rounded ${mode === 'two' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('two')}>Two Color</button>
    </div>
  );
};

export default LightingMode;

