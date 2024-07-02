// src/components/Status.tsx

/*
This component displays the status and battery level of the robot.
It shows a check icon if the robot is connected and a cross icon if it is disconnected.
The battery level is displayed as a percentage and a visual bar that changes color based on the battery level.
*/

import React, { useState, useEffect, useRef } from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

const Status: React.FC = () => {
  const [status, setStatus] = useState('Disconnected');
  const [batteryLevel, setBatteryLevel] = useState(0);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
      setStatus('Connected');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.battery !== undefined) {
        setBatteryLevel(data.battery);
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
      setStatus('Disconnected');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
      setStatus('Disconnected');
    };

    // Cleanup on unmount
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  // Determine the class for the battery bar color based on battery level
  const getBatteryClass = (level: number) => {
    if (level === 0) return 'battery-empty';
    if (level > 75) return 'bg-green-500';
    if (level > 50) return 'bg-yellow-500';
    if (level > 20) return 'neon-blue';
    return 'bg-red-500';
  };

  const batteryClass = getBatteryClass(batteryLevel);
  const batteryBarClass = `h-4 rounded ${batteryClass}`;

  return (
    <div className="flex items-center space-x-4">
      {/* Display the status with an icon */}
      <div className="flex items-center">
        Status: {status === 'Connected' ? (
          <FaCheckCircle data-testid="status-icon" className="text-green-500 ml-2" />
        ) : (
          <FaTimesCircle data-testid="status-icon" className="text-red-500 ml-2" />
        )}
      </div>
      {/* Display the battery level as a bar and percentage */}
      <div className="flex items-center">
        Battery:
        <div className="ml-2 w-32 battery-container">
          <div
            className={batteryBarClass}
            style={{ width: `${batteryLevel}%` }}
          ></div>
        </div>
        <span className="ml-2">{batteryLevel}%</span>
      </div>
    </div>
  );
};

export default Status;
