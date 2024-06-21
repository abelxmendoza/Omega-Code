// File: /Omega-Code/ui/robot-controller-ui/components/ControlButtons.tsx
import React from 'react';

const ControlButtons: React.FC<{ onStart: () => void, onStop: () => void, onApply: () => void }> = ({ onStart, onStop, onApply }) => {
  return (
    <div className="flex space-x-4">
      <button className="bg-green-500 text-white p-2 rounded" onClick={onStart}>Start</button>
      <button className="bg-red-500 text-white p-2 rounded" onClick={onStop}>Stop</button>
      <button className="bg-blue-500 text-white p-2 rounded" onClick={onApply}>Apply Settings</button>
    </div>
  );
};

export default ControlButtons;
