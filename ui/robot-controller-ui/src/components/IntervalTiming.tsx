// File: /Omega-Code/ui/robot-controller-ui/components/IntervalTiming.tsx
import React, { useState } from 'react';

const IntervalTiming: React.FC<{ onSetInterval: (interval: number) => void }> = ({ onSetInterval }) => {
  const [interval, setInterval] = useState(500);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newInterval = Number(e.target.value);
    setInterval(newInterval);
    onSetInterval(newInterval);
  };

  return (
    <div className="flex items-center space-x-2">
      <label htmlFor="interval">Interval (ms):</label>
      <input id="interval" type="number" value={interval} onChange={handleChange} className="p-2 border rounded" />
    </div>
  );
};

export default IntervalTiming;
