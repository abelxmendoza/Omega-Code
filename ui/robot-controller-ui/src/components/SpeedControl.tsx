import React from 'react';

const SpeedControl: React.FC = () => {
  return (
    <div className="flex items-center space-x-4">
      <label htmlFor="speed">Speed</label>
      <input id="speed" type="range" min="1" max="100" defaultValue="50" />
    </div>
  );
};

export default SpeedControl;
