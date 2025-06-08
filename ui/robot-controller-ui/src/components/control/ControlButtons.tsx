// File: /Omega-Code/ui/robot-controller-u./components/control/ControlButtons.tsx

/*
This component provides three control buttons: Start, Stop, and Apply Settings.
Each button triggers a corresponding callback function passed down from the parent component.
These buttons are used to control various aspects of the robot's operation.
*/

import React from 'react';

const ControlButtons: React.FC<{ onStart: () => void, onStop: () => void, onApply: () => void }> = ({ onStart, onStop, onApply }) => {
  return (
    <div className="flex space-x-4">
      {/* Start button */}
      <button 
        className="bg-green-500 text-white p-2 rounded" 
        onClick={onStart}
      >
        Start
      </button>
      
      {/* Stop button */}
      <button 
        className="bg-red-500 text-white p-2 rounded" 
        onClick={onStop}
      >
        Stop
      </button>
      
      {/* Apply Settings button */}
      <button 
        className="bg-blue-500 text-white p-2 rounded" 
        onClick={onApply}
      >
        Apply Settings
      </button>
    </div>
  );
};

export default ControlButtons;
