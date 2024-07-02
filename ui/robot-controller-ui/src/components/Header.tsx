import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

const Header: React.FC<{ isConnected: boolean; batteryLevel: number }> = ({ isConnected, batteryLevel }) => {
  const batteryClass = batteryLevel > 20 ? 'bg-blue-500' : 'bg-red-500';
  const batteryStyle = batteryLevel > 20 ? 'neon-blue' : 'black';
  const batteryBarClass = `h-4 rounded ${batteryClass}`;

  return (
    <div className="flex justify-between items-center bg-gray-800 text-white p-4 sticky top-0 z-10">
      <div className="text-lg font-bold">Robot Controller</div>
      <div className="flex items-center space-x-4">
        <div className="flex items-center">
          Status: {isConnected ? (
            <FaCheckCircle data-testid="status-icon" className="text-green-500 ml-2" />
          ) : (
            <FaTimesCircle data-testid="status-icon" className="text-red-500 ml-2" />
          )}
          <span className="ml-2">{isConnected ? 'Connected' : 'Disconnected'}</span>
        </div>
        <div className="flex items-center">
          Battery:
          <div className={`ml-2 w-32 ${batteryStyle}`}>
            <div
              className={batteryBarClass}
              style={{ width: `${batteryLevel}%` }}
            ></div>
          </div>
          <span className="ml-2">{batteryLevel}%</span>
        </div>
      </div>
    </div>
  );
};

export default Header;
