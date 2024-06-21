import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

interface StatusProps {
  status: string;
  battery: number;
}

const Status: React.FC<StatusProps> = ({ status, battery }) => {
  const batteryClass = battery > 20 ? 'bg-blue-500' : 'bg-red-500';
  const batteryStyle = battery > 20 ? 'neon-blue' : 'black';
  const batteryBarClass = `h-4 rounded ${batteryClass}`;

  return (
    <div className="flex items-center space-x-4">
      <div className="flex items-center">
        Status: {status === 'Connected' ? <FaCheckCircle className="text-green-500 ml-2" /> : <FaTimesCircle className="text-red-500 ml-2" />}
      </div>
      <div className="flex items-center">
        Battery:
        <div className={`ml-2 w-32 ${batteryStyle}`}>
          <div
            className={batteryBarClass}
            style={{ width: `${battery}%` }}
          ></div>
        </div>
        <span className="ml-2">{battery}%</span>
      </div>
    </div>
  );
};

export default Status;

