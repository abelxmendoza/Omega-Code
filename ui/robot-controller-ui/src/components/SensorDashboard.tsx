/*
# File: /src/components/SensorDashboard.tsx

Sensor Dashboard Component

This component displays real-time sensor data for line tracking and ultrasonic distances
in multiple units (cm, meters, inches, feet) using a WebSocket connection.
*/

import React, { useEffect, useState, useRef } from 'react';

interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

const SensorDashboard: React.FC = () => {
  const [lineTrackingData, setLineTrackingData] = useState({ IR01: 0, IR02: 0, IR03: 0 });
  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0,
    distance_m: 0,
    distance_inch: 0,
    distance_feet: 0,
  });
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://192.168.1.134:8080/ws'); // Update IP as needed

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.lineTracking) {
        setLineTrackingData(data.lineTracking);
      }
      if (data.distance_cm !== undefined) {
        setUltrasonicData(data);
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Cleanup on unmount
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md">
      <h2 className="text-lg font-bold mb-4">Sensor Dashboard</h2>
      <div className="sensor-container grid grid-cols-1 md:grid-cols-2 gap-4">
        <div className="line-tracking bg-gray-900 p-3 rounded-md shadow">
          <strong className="block text-sm mb-2">Line Tracking:</strong>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>
        <div className="ultrasonic-distance bg-gray-900 p-3 rounded-md shadow">
          <strong className="block text-sm mb-2">Ultrasonic Distance:</strong>
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