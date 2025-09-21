import React from 'react';

interface IntervalTimingProps {
  interval?: number;
  onIntervalChange?: (interval: number) => void;
}

const IntervalTiming: React.FC<IntervalTimingProps> = ({ 
  interval = 1000, 
  onIntervalChange 
}) => {
  return (
    <div>
      <label>Interval (ms):</label>
      <input
        type="number"
        value={interval}
        onChange={(e) => onIntervalChange?.(Number(e.target.value))}
      />
    </div>
  );
};

export default IntervalTiming;
