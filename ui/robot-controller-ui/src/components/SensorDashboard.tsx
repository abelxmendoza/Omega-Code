import React, { useEffect, useState, useRef } from 'react';

const SensorDashboard: React.FC = () => {
  const [lineTrackingData, setLineTrackingData] = useState({ IR01: 0, IR02: 0, IR03: 0 });
  const [ultrasonicDistance, setUltrasonicDistance] = useState(0);
  const ws = useRef<WebSocket | null>(null);


  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.lineTracking) {
        setLineTrackingData(data.lineTracking);
      }
      if (data.ultrasonicDistance !== undefined) {
        setUltrasonicDistance(data.ultrasonicDistance);
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
    <div className="sensor-dashboard">
      <h2>Sensor Dashboard</h2>
      <div className="sensor-container">
        <div className="line-tracking">
          <strong>Line Tracking:</strong>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>
        <div className="ultrasonic-distance">
          <strong>Ultrasonic Distance:</strong>
          <p>Distance: {ultrasonicDistance} cm</p>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;
