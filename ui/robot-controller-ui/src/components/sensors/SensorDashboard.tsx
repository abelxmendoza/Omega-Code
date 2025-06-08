/*
# File: sr./components/sensors/SensorDashboard.tsx

# Summary:
This component serves as a dashboard displaying real-time sensor data:
- **Line tracking** status.
- **Ultrasonic sensor distances** in multiple units (cm, m, in, ft).

- Establishes a WebSocket connection to the server.
- Updates data dynamically for both line tracking and ultrasonic sensors.
*/

import React, { useEffect, useState, useRef } from 'react';

interface LineTrackingData {
  IR01: number;
  IR02: number;
  IR03: number;
}

interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

const SensorDashboard: React.FC = () => {
  // State to store line tracking sensor data
  const [lineTrackingData, setLineTrackingData] = useState<LineTrackingData>({
    IR01: 0,
    IR02: 0,
    IR03: 0,
  });

  // State to store ultrasonic sensor data
  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0,
    distance_m: 0,
    distance_inch: 0,
    distance_feet: 0,
  });

  const ws = useRef<WebSocket | null>(null); // WebSocket reference for live updates

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://192.168.1.134:8080/ultrasonic'); // Update the IP address as needed

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data); // Parse incoming JSON data
      if (data.lineTracking) {
        setLineTrackingData(data.lineTracking); // Update line tracking data
      }
      if (data.distance_cm !== undefined) {
        setUltrasonicData(data); // Update ultrasonic sensor data
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Cleanup on component unmount
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md max-w-6xl mx-auto">
      {/* Dashboard Header */}
      <h2 className="text-lg font-bold mb-4 text-center">Sensor Dashboard</h2>

      {/* Sensor Data Grid */}
      <div className="sensor-container grid grid-cols-1 sm:grid-cols-2 gap-6">
        {/* Line Tracking Sensor Data */}
        <div className="line-tracking bg-gray-900 p-4 rounded-lg shadow-lg">
          <strong className="block text-sm mb-2 underline">Line Tracking:</strong>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>

        {/* Ultrasonic Sensor Data */}
        <div className="ultrasonic-distance bg-gray-900 p-4 rounded-lg shadow-lg">
          <strong className="block text-sm mb-2 underline">Ultrasonic Distance:</strong>
          <p>Distance: {ultrasonicData.distance_cm} cm</p>
          <p>Distance: {ultrasonicData.distance_m.toFixed(2)} m</p>
          <p>Distance: {ultrasonicData.distance_inch.toFixed(2)} inches</p>
          <p>Distance: {ultrasonicData.distance_feet.toFixed(2)} feet</p>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;
