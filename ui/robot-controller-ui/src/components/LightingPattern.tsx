// File: /Omega-Code/ui/robot-controller-ui/components/LightingPattern.tsx
import React, { useState } from 'react';

const LightingPattern: React.FC<{ onSelectPattern: (pattern: string) => void }> = ({ onSelectPattern }) => {
  const [pattern, setPattern] = useState('static');

  const handleChange = (newPattern: string) => {
    setPattern(newPattern);
    onSelectPattern(newPattern);
  };

  return (
    <div className="flex space-x-4">
      <button className={`p-2 rounded ${pattern === 'static' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('static')}>Static</button>
      <button className={`p-2 rounded ${pattern === 'blink' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('blink')}>Blink</button>
      <button className={`p-2 rounded ${pattern === 'fade' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('fade')}>Fade</button>
      <button className={`p-2 rounded ${pattern === 'chase' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('chase')}>Chase</button>
      <button className={`p-2 rounded ${pattern === 'rainbow' ? 'bg-blue-500 text-white' : 'bg-gray-300'}`} onClick={() => handleChange('rainbow')}>Rainbow</button>
    </div>
  );
};

export default LightingPattern;

