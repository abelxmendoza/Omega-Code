import React from 'react';

interface SensorDashboardProps {
  sensors?: Record<string, any>;
}

const SensorDashboard: React.FC<SensorDashboardProps> = ({ 
  sensors = {} 
}) => {
  return (
    <div>
      <h3>Sensor Dashboard</h3>
      <div>
        {Object.entries(sensors).map(([key, value]) => (
          <div key={key}>
            {key}: {JSON.stringify(value)}
          </div>
        ))}
      </div>
    </div>
  );
};

export default SensorDashboard;
