import React, { useState, useEffect, useRef } from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

const Header: React.FC = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [batteryLevel, setBatteryLevel] = useState(0);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
      setIsConnected(true);
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.battery !== undefined) {
        setBatteryLevel(data.battery);
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
      setIsConnected(false);
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
      setIsConnected(false);
    };

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  const batteryClass = batteryLevel > 20 ? 'bg-blue-500 neon-blue' : 'bg-red-500';
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
