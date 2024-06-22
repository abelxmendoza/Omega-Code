// src/components/Header.tsx

/*
This component renders the header for the robot control interface.
It displays the application title, connection status with an icon, and the battery level as a percentage and a visual bar.
*/

import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

const Header: React.FC<{ isConnected: boolean; batteryLevel: number }> = ({ isConnected, batteryLevel }) => {
  // Determine the class for the battery bar color based on battery level
  const batteryClass = batteryLevel > 20 ? 'bg-blue-500' : 'bg-red-500';
  const batteryStyle = batteryLevel > 20 ? 'neon-blue' : 'black';
  const batteryBarClass = `h-4 rounded ${batteryClass}`;

  return (
    <div className="flex justify-between items-center bg-gray-800 text-white p-4 sticky top-0 z-10">
      {/* Application title */}
      <div className="text-lg font-bold">Robot Controller</div>
      <div className="flex items-center space-x-4">
        {/* Display the connection status with an icon */}
        <div className="flex items-center">
          Status: {isConnected ? (
            <FaCheckCircle className="text-green-500 ml-2" />
          ) : (
            <FaTimesCircle className="text-red-500 ml-2" />
          )}
        </div>
        {/* Display the battery level as a bar and percentage */}
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
