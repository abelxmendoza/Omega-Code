import React from 'react';

interface UltrasonicSensorStatusProps {
  distance?: number;
  status?: 'active' | 'inactive';
}

const UltrasonicSensorStatus: React.FC<UltrasonicSensorStatusProps> = ({ 
  distance = 0, 
  status = 'inactive' 
}) => {
  return (
    <div>
      <h3>Ultrasonic Sensor Status</h3>
      <div>Distance: {distance}cm</div>
      <div>Status: {status}</div>
    </div>
  );
};

export default UltrasonicSensorStatus;
