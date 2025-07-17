/*
# File: src/components/sensors/UltrasonicSensorStatus.tsx

# Summary:
This React component displays the ultrasonic sensor's distance readings in multiple units:
- **centimeters (cm)**
- **meters (m)**
- **inches (in)**
- **feet (ft)**

Functionality:
- Connects to a WebSocket server to receive real-time sensor data.
- Dynamically updates the displayed values as new data arrives.
- Cleans up the WebSocket connection on component unmount.
*/

import React, { useState, useEffect, useRef } from 'react';

interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

const UltrasonicSensorStatus: React.FC = () => {
  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0,
    distance_m: 0,
    distance_inch: 0,
    distance_feet: 0,
  });
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket(process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC as string);

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
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

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  return (
    <div style={{ margin: '20px', fontFamily: 'Arial, sans-serif' }}>
      <h3>Ultrasonic Sensor Status</h3>
      <p>Distance: <strong>{ultrasonicData.distance_cm} cm</strong></p>
      <p>Distance: <strong>{ultrasonicData.distance_m.toFixed(2)} m</strong></p>
      <p>Distance: <strong>{ultrasonicData.distance_inch.toFixed(2)} inches</strong></p>
      <p>Distance: <strong>{ultrasonicData.distance_feet.toFixed(2)} feet</strong></p>
    </div>
  );
};

export default UltrasonicSensorStatus;
