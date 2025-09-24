import React, { useState, useEffect, useRef } from 'react';
import { useRobustWebSocket } from '../utils/RobustWebSocket';
import { envConfig } from '@/config/environment';
import ErrorBoundary from './ErrorBoundary';

interface SensorDashboardProps {
  sensors?: Record<string, any>;
  onSensorUpdate?: (sensorId: string, value: any) => void;
}

const SensorDashboard: React.FC<SensorDashboardProps> = ({ 
  sensors = {},
  onSensorUpdate
}) => {
  const [lineTrackerData, setLineTrackerData] = useState({ left: 0, center: 0, right: 0 });
  const [ultrasonicData, setUltrasonicData] = useState({ distance: 0, status: 'disconnected' });

  // Use robust WebSocket connections
  const lineTrackerWs = useRobustWebSocket({
    url: envConfig.wsUrls.lineTracker[0] || 'ws://localhost:8090/line-tracker',
    onMessage: (data) => {
      if (data?.lineTracking) {
        setLineTrackerData(data.lineTracking);
      } else if (data?.type === 'sample' && data?.lineTracking) {
        setLineTrackerData(data.lineTracking);
      }
      onSensorUpdate?.('lineTracker', data);
    },
    onError: (error) => {
      console.error('Line tracker WebSocket error:', error);
    }
  });
  
  const ultrasonicWs = useRobustWebSocket({
    url: envConfig.wsUrls.ultrasonic[0] || 'ws://localhost:8080/ultrasonic',
    onMessage: (data) => {
      if (data?.distance !== undefined) {
        setUltrasonicData({ distance: data.distance, status: 'connected' });
      } else if (data?.type === 'sample' && data?.distance !== undefined) {
        setUltrasonicData({ distance: data.distance, status: 'connected' });
      }
      onSensorUpdate?.('ultrasonic', data);
    },
    onError: (error) => {
      console.error('Ultrasonic WebSocket error:', error);
      setUltrasonicData(prev => ({ ...prev, status: 'error' }));
    }
  });


  const getStatusColor = (status: string) => {
    switch (status) {
      case 'connected': return 'bg-green-500';
      case 'connecting': return 'bg-yellow-500';
      case 'disconnected': return 'bg-red-500';
      case 'error': return 'bg-red-600';
      default: return 'bg-gray-500';
    }
  };

  const getStatusText = (status: string) => {
    switch (status) {
      case 'connected': return 'Connected';
      case 'connecting': return 'Connecting...';
      case 'disconnected': return 'Disconnected';
      case 'error': return 'Error';
      default: return 'Unknown';
    }
  };

  return (
    <ErrorBoundary>
      <div className="p-4 bg-gray-800 rounded-lg">
        <h3 className="text-white font-bold mb-4">Sensor Dashboard</h3>
        
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-3 sm:gap-4">
          {/* Line Tracker Sensor */}
          <ErrorBoundary>
            <div className="bg-gray-700 p-3 rounded">
              <div className="flex items-center justify-between mb-2">
                <div className="text-white font-semibold">Line Tracker</div>
                <div className={`px-2 py-1 rounded-full text-xs ${getStatusColor(lineTrackerWs.connectionStatus)}`}>
                  {getStatusText(lineTrackerWs.connectionStatus)}
                </div>
              </div>
              <div className="text-gray-300 text-sm space-y-1">
                <div>Left: <span className={lineTrackerData.left ? 'text-green-400' : 'text-red-400'}>{lineTrackerData.left}</span></div>
                <div>Center: <span className={lineTrackerData.center ? 'text-green-400' : 'text-red-400'}>{lineTrackerData.center}</span></div>
                <div>Right: <span className={lineTrackerData.right ? 'text-green-400' : 'text-red-400'}>{lineTrackerData.right}</span></div>
              </div>
              {lineTrackerWs.connectionStatus === 'error' && (
                <button 
                  className="mt-2 px-2 py-1 bg-blue-500 text-white text-xs rounded hover:bg-blue-600"
                  onClick={() => lineTrackerWs.connect()}
                >
                  Retry Connection
                </button>
              )}
            </div>
          </ErrorBoundary>

          {/* Ultrasonic Sensor */}
          <ErrorBoundary>
            <div className="bg-gray-700 p-3 rounded">
              <div className="flex items-center justify-between mb-2">
                <div className="text-white font-semibold">Ultrasonic</div>
                <div className={`px-2 py-1 rounded-full text-xs ${getStatusColor(ultrasonicWs.connectionStatus)}`}>
                  {getStatusText(ultrasonicWs.connectionStatus)}
                </div>
              </div>
              <div className="text-gray-300 text-sm">
                <div>Distance: <span className="text-blue-400">{ultrasonicData.distance} cm</span></div>
                <div>Status: <span className={ultrasonicData.status === 'success' ? 'text-green-400' : 'text-red-400'}>{ultrasonicData.status}</span></div>
              </div>
              {ultrasonicWs.connectionStatus === 'error' && (
                <button 
                  className="mt-2 px-2 py-1 bg-blue-500 text-white text-xs rounded hover:bg-blue-600"
                  onClick={() => ultrasonicWs.connect()}
                >
                  Retry Connection
                </button>
              )}
            </div>
          </ErrorBoundary>

          {/* Other sensors */}
          {Object.entries(sensors).map(([key, value]) => (
            <ErrorBoundary key={key}>
              <div className="bg-gray-700 p-3 rounded">
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
            </ErrorBoundary>
          ))}
          
          {Object.keys(sensors).length === 0 && 
            lineTrackerWs.connectionStatus === 'disconnected' && 
            ultrasonicWs.connectionStatus === 'disconnected' && (
            <div className="col-span-2 text-center text-gray-400 py-8">
              No sensors connected
            </div>
          )}
        </div>
      </div>
    </ErrorBoundary>
  );
};

export default SensorDashboard;
