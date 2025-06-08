/*
# File: sr./components/sensors/UltrasonicSensorStatus.tsx

# Summary:
This component displays the ultrasonic sensor distance in **centimeters (cm)** along with other units:
- **meters (m)**
- **inches (in)**
- **feet (ft)**

- Establishes a WebSocket connection to the server.
- Updates the distance dynamically as it receives data from the WebSocket.
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
    ws.current = new WebSocket('ws://192.168.1.134:8080/ultrasonic'); // Update IP as needed

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

    // Cleanup on unmount
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