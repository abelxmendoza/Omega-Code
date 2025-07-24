// File: src/components/sensors/SensorDashboard.tsx

/**
 * SensorDashboard Component
 * 
 * Displays real-time sensor data received over WebSocket:
 * - Line Tracking Sensor (IR01, IR02, IR03)
 * - Ultrasonic Sensor (distance in cm, m, inches, feet)
 * 
 * Automatically connects to the backend server via WebSocket
 * and updates UI upon receiving sensor events.
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
  const [lineTrackingData, setLineTrackingData] = useState<LineTrackingData>({
    IR01: 0,
    IR02: 0,
    IR03: 0,
  });

  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0,
    distance_m: 0,
    distance_inch: 0,
    distance_feet: 0,
  });

  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    const LOCAL_WS_URL = process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL || 'ws://192.168.1.134:8080/ultrasonic';
    const TAILSCALE_WS_URL = process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE || 'ws://100.93.225.61:8080/ultrasonic';

    // Prioritize Tailscale, fallback to local
    const selectedWSUrl = TAILSCALE_WS_URL || LOCAL_WS_URL;
    ws.current = new WebSocket(selectedWSUrl);

    ws.current.onopen = () => {
      console.log(`🔌 WebSocket connected: ${selectedWSUrl}`);
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
      console.log('❌ WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('⚠️ WebSocket error:', error);
    };

    return () => {
      ws.current?.close();
    };
  }, []);

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md max-w-6xl mx-auto">
      <h2 className="text-lg font-bold mb-4 text-center">Sensor Dashboard</h2>

      <div className="sensor-container grid grid-cols-1 sm:grid-cols-2 gap-6">
        <div className="line-tracking bg-gray-900 p-4 rounded-lg shadow-lg">
          <strong className="block text-sm mb-2 underline">Line Tracking Sensor</strong>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>

        <div className="ultrasonic-distance bg-gray-900 p-4 rounded-lg shadow-lg">
          <strong className="block text-sm mb-2 underline">Ultrasonic Distance</strong>
          <p>{ultrasonicData.distance_cm} cm</p>
          <p>{ultrasonicData.distance_m.toFixed(2)} m</p>
          <p>{ultrasonicData.distance_inch.toFixed(2)} inches</p>
          <p>{ultrasonicData.distance_feet.toFixed(2)} feet</p>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;

