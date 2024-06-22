// src/components/Status.tsx

/*
This component displays the status and battery level of the robot.
It shows a check icon if the robot is connected and a cross icon if it is disconnected.
The battery level is displayed as a percentage and a visual bar that changes color based on the battery level.
*/

import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

interface StatusProps {
  status: string;
  battery: number;
}

const Status: React.FC<StatusProps> = ({ status, battery }) => {
  // Determine the class for the battery bar color based on battery level
  const batteryClass = battery > 20 ? 'bg-blue-500' : 'bg-red-500';
  const batteryStyle = battery > 20 ? 'neon-blue' : 'black';
  const batteryBarClass = `h-4 rounded ${batteryClass}`;

  return (
    <div className="flex items-center space-x-4">
      {/* Display the status with an icon */}
      <div className="flex items-center">
        Status: {status === 'Connected' ? (
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
            style={{ width: `${battery}%` }}
          ></div>
        </div>
        <span className="ml-2">{battery}%</span>
      </div>
    </div>
  );
};

export default Status;
