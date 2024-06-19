import React from 'react';

interface StatusProps {
  status: string;
  battery: number;
}

const Status: React.FC<StatusProps> = ({ status, battery }) => {
  return (
    <div className="flex items-center space-x-4">
      <span>Status: {status}</span>
      <span>Battery: {battery}%</span>
    </div>
  );
};

export default Status;
