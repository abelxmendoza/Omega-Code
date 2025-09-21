import React from 'react';

interface SensorDashboardProps {
  sensors?: Record<string, any>;
  onSensorUpdate?: (sensorId: string, value: any) => void;
}

const SensorDashboard: React.FC<SensorDashboardProps> = ({ 
  sensors = {},
  onSensorUpdate
}) => {
  return (
    <div className="p-4 bg-gray-800 rounded-lg">
      <h3 className="text-white font-bold mb-4">Sensor Dashboard</h3>
      
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3 sm:gap-4">
        {Object.entries(sensors).map(([key, value]) => (
          <div key={key} className="bg-gray-700 p-3 rounded">
            <div className="text-white font-semibold mb-2">{key}</div>
            <div className="text-gray-300 text-sm">
              {typeof value === 'object' ? JSON.stringify(value) : String(value)}
            </div>
            {onSensorUpdate && (
              <button 
                className="mt-2 px-2 py-1 bg-blue-500 text-white text-xs rounded hover:bg-blue-600"
                onClick={() => onSensorUpdate(key, value)}
              >
                Update
              </button>
            )}
          </div>
        ))}
        
        {Object.keys(sensors).length === 0 && (
          <div className="col-span-2 text-center text-gray-400 py-8">
            No sensors connected
          </div>
        )}
      </div>
    </div>
  );
};

export default SensorDashboard;
