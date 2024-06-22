// File: /Omega-Code/ui/robot-controller-ui/components/IntervalTiming.tsx

/*
This component allows the user to set the interval timing for LED patterns.
It provides an input field where the user can specify the interval in milliseconds.
The selected interval is stored in the component's state and sent back to the parent component through the onSetInterval callback.
*/

import React, { useState } from 'react';

const IntervalTiming: React.FC<{ onSetInterval: (interval: number) => void }> = ({ onSetInterval }) => {
  const [interval, setInterval] = useState(500); // State to store the interval timing

  // Handle change in the input field and update the state and parent component
  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newInterval = Number(e.target.value);
    setInterval(newInterval);
    onSetInterval(newInterval);
  };

  return (
    <div className="flex items-center space-x-2">
      <label htmlFor="interval">Interval (ms):</label>
      <input 
        id="interval" 
        type="number" 
        value={interval} 
        onChange={handleChange} 
        className="p-2 border rounded" 
      />
    </div>
  );
};

export default IntervalTiming;
