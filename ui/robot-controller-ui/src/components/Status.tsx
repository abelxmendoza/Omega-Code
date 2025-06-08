/*
# File: /src/components/Status.tsx
# Summary:
Displays the status and battery level of the robot.
Indicates connection status using icons and visualizes battery level with a percentage and color-coded bar.
Automatically attempts reconnection if the WebSocket disconnects.
*/

import React, { useState, useEffect, useRef } from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

const Status: React.FC = () => {
  const [status, setStatus] = useState<'Connected' | 'Disconnected'>('Disconnected'); // Connection status
  const [batteryLevel, setBatteryLevel] = useState(0); // Battery level (percentage)
  const ws = useRef<WebSocket | null>(null); // WebSocket instance reference
  const wsUrl = 'ws://localhost:8080/ws'; // WebSocket URL

  useEffect(() => {
    // Function to establish and manage WebSocket connection
    const connectWebSocket = () => {
      ws.current = new WebSocket(wsUrl);

      // WebSocket opened successfully
      ws.current.onopen = () => {
        console.log('[WebSocket] Connection established');
        setStatus('Connected');
      };

      // Handle incoming WebSocket messages
      ws.current.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.battery !== undefined) {
            setBatteryLevel(data.battery);
          }
        } catch (error) {
          console.error('[WebSocket] Error parsing message:', error);
        }
      };

      // WebSocket connection closed
      ws.current.onclose = () => {
        console.log('[WebSocket] Connection closed');
        setStatus('Disconnected');
        setTimeout(() => {
          console.log('[WebSocket] Attempting to reconnect...');
          connectWebSocket(); // Reconnect after delay
        }, 2000); // Retry every 2 seconds
      };

      // WebSocket error occurred
      ws.current.onerror = (error) => {
        console.error('[WebSocket] Error:', error);
        setStatus('Disconnected');
      };
    };

    // Initiate WebSocket connection
    connectWebSocket();

    // Cleanup WebSocket on component unmount
    return () => {
      if (ws.current) {
        ws.current.close();
        console.log('[WebSocket] Connection closed during cleanup');
      }
    };
  }, [wsUrl]);

  // Determine the CSS class for the battery bar based on the battery level
  const getBatteryClass = (level: number) => {
    if (level > 75) return 'bg-green-500'; // High battery level
    if (level > 50) return 'bg-yellow-500'; // Medium battery level
    if (level > 20) return 'neon-blue'; // Low battery level
    return 'bg-red-500'; // Critical battery level
  };

  // CSS class for the battery bar
  const batteryBarClass = `h-4 rounded ${getBatteryClass(batteryLevel)}`;

  return (
    <div className="flex items-center space-x-4">
      {/* Connection status indicator */}
      <div className="flex items-center">
        Status:
        {status === 'Connected' ? (
          <FaCheckCircle data-testid="status-icon" className="text-green-500 ml-2" />
        ) : (
          <FaTimesCircle data-testid="status-icon" className="text-red-500 ml-2" />
        )}
        <span className="ml-2">{status}</span>
      </div>
      {/* Battery level indicator */}
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
