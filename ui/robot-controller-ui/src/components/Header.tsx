/*
# File: /src/components/Header.tsx
# Summary:
Displays the application header with connection status and battery level indicators.
Dynamically updates based on WebSocket connection status and backend data.
*/

import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

// Define the props for the Header component
interface HeaderProps {
  isConnected: boolean; // WebSocket connection status
  batteryLevel: number; // Current battery level
}

const Header: React.FC<HeaderProps> = ({ isConnected, batteryLevel }) => {
  /**
   * Determines the CSS class for the battery bar based on the battery level.
   * @param level - The battery level percentage.
   * @returns A string representing the CSS class for the battery bar.
   */
  const getBatteryClass = (level: number): string => {
    if (level > 75) return 'bg-green-500'; // High battery level
    if (level > 50) return 'bg-yellow-500'; // Medium battery level
    if (level > 20) return 'bg-blue-500 neon-blue'; // Low battery level
    return 'bg-red-500'; // Critical battery level
  };

  const batteryBarClass = `h-4 rounded ${getBatteryClass(batteryLevel)}`;

  return (
    <div className="flex justify-between items-center bg-gray-800 text-white p-4 sticky top-0 z-10 shadow-md">
      {/* Application title */}
      <div className="text-lg font-bold">Robot Controller</div>

      {/* Status and battery indicators */}
      <div className="flex items-center space-x-4">
        {/* Connection status */}
        <div className="flex items-center">
          <span>Status:</span>
          {isConnected ? (
            <FaCheckCircle data-testid="status-icon" className="text-green-500 ml-2" />
          ) : (
            <FaTimesCircle data-testid="status-icon" className="text-red-500 ml-2" />
          )}
          <span className="ml-2">{isConnected ? 'Connected' : 'Disconnected'}</span>
        </div>

        {/* Battery level */}
        <div className="flex items-center">
          <span>Battery:</span>
          <div className="ml-2 w-32 battery-container">
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
