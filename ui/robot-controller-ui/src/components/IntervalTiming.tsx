import React from 'react';

interface IntervalTimingProps {
  interval?: number;
  onIntervalChange?: (interval: number) => void;
  onSetInterval?: (interval: number) => void; // For test compatibility
}

const IntervalTiming: React.FC<IntervalTimingProps> = ({ 
  interval = 1000, 
  onIntervalChange,
  onSetInterval
}) => {
  const handleChange = (value: number) => {
    onIntervalChange?.(value);
    onSetInterval?.(value); // Support both prop names
  };

  return (
    <div>
      <label htmlFor="interval-input">Interval (ms):</label>
      <input
        id="interval-input"
        type="number"
        value={interval}
        onChange={(e) => handleChange(Number(e.target.value))}
      />
    </div>
  );
};

export default IntervalTiming;
