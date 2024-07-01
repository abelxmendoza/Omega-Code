import React, { useEffect, useState } from 'react';

const SensorDashboard: React.FC = () => {
  const [lineTrackingData, setLineTrackingData] = useState({ IR01: 0, IR02: 0, IR03: 0 });
  const [ultrasonicDistance, setUltrasonicDistance] = useState(0);

  useEffect(() => {
    const fetchLineTrackingData = async () => {
      try {
        const response = await fetch('https://localhost:8080/line-tracking');
        if (!response.ok) throw new Error('Network response was not ok');
        const data = await response.json();
        setLineTrackingData(data);
      } catch (error) {
        console.error('Error fetching line tracking data:', error);
      }
    };

    const fetchUltrasonicData = async () => {
      try {
        const response = await fetch('https://localhost:8080/ultrasonic-sensor');
        if (!response.ok) throw new Error('Network response was not ok');
        const data = await response.json();
        setUltrasonicDistance(data.distance || 0);
      } catch (error) {
        console.error('Error fetching ultrasonic data:', error);
      }
    };

    fetchLineTrackingData();
    fetchUltrasonicData();

    const intervalId = setInterval(() => {
      fetchLineTrackingData();
      fetchUltrasonicData();
    }, 1000);

    return () => clearInterval(intervalId);
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

