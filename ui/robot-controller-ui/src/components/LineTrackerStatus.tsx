import React from 'react';

interface LineTrackerStatusProps {
  status?: 'active' | 'inactive' | 'error';
  onToggle?: () => void;
}

const LineTrackerStatus: React.FC<LineTrackerStatusProps> = ({ 
  status = 'inactive', 
  onToggle 
}) => {
  return (
    <div>
      <h3>Line Tracker Status</h3>
      <div>Status: {status}</div>
      <button onClick={onToggle}>Toggle</button>
    </div>
  );
};

export default LineTrackerStatus;
