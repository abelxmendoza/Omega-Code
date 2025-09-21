import React, { useState, useEffect } from 'react';

interface SensorData {
  value: number;
  unit: string;
  timestamp: number;
  min?: number;
  max?: number;
}

interface SensorVisualizationProps {
  sensorId: string;
  data: SensorData;
  type?: 'gauge' | 'line' | 'bar' | 'radial';
  color?: string;
  size?: 'small' | 'medium' | 'large';
  showHistory?: boolean;
  historyLength?: number;
}

const SensorVisualization: React.FC<SensorVisualizationProps> = ({
  sensorId,
  data,
  type = 'gauge',
  color = '#3b82f6',
  size = 'medium',
  showHistory = false,
  historyLength = 10
}) => {
  const [history, setHistory] = useState<SensorData[]>([]);

  useEffect(() => {
    setHistory(prev => {
      const newHistory = [...prev, data];
      return newHistory.slice(-historyLength);
    });
  }, [data, historyLength]);

  const getSizeClasses = () => {
    switch (size) {
      case 'small': return 'w-24 h-24';
      case 'large': return 'w-32 h-32';
      default: return 'w-28 h-28';
    }
  };

  const getPercentage = () => {
    if (data.min !== undefined && data.max !== undefined) {
      return ((data.value - data.min) / (data.max - data.min)) * 100;
    }
    return Math.min((data.value / 100) * 100, 100);
  };

  const renderGauge = () => {
    const percentage = getPercentage();
    const circumference = 2 * Math.PI * 45; // radius = 45
    const strokeDasharray = circumference;
    const strokeDashoffset = circumference - (percentage / 100) * circumference;

    return (
      <div className={`${getSizeClasses()} relative`}>
        <svg className="w-full h-full transform -rotate-90">
          {/* Background circle */}
          <circle
            cx="50%"
            cy="50%"
            r="45"
            stroke="currentColor"
            strokeWidth="8"
            fill="none"
            className="text-gray-600"
          />
          {/* Progress circle */}
          <circle
            cx="50%"
            cy="50%"
            r="45"
            stroke={color}
            strokeWidth="8"
            fill="none"
            strokeDasharray={strokeDasharray}
            strokeDashoffset={strokeDashoffset}
            className="transition-all duration-500 ease-out"
            strokeLinecap="round"
          />
        </svg>
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="text-center">
            <div className="text-white font-bold text-lg">
              {data.value.toFixed(1)}
            </div>
            <div className="text-gray-400 text-xs">
              {data.unit}
            </div>
          </div>
        </div>
      </div>
    );
  };

  const renderLineChart = () => {
    if (history.length < 2) return null;

    const maxValue = Math.max(...history.map(h => h.value));
    const minValue = Math.min(...history.map(h => h.value));
    const range = maxValue - minValue || 1;

    const points = history.map((point, index) => {
      const x = (index / (history.length - 1)) * 100;
      const y = 100 - ((point.value - minValue) / range) * 100;
      return `${x},${y}`;
    }).join(' ');

    return (
      <div className={`${getSizeClasses()} relative`}>
        <svg className="w-full h-full">
          <polyline
            points={points}
            fill="none"
            stroke={color}
            strokeWidth="2"
            className="transition-all duration-300"
          />
          {/* Current value indicator */}
          <circle
            cx={100}
            cy={100 - ((data.value - minValue) / range) * 100}
            r="3"
            fill={color}
            className="animate-pulse"
          />
        </svg>
        <div className="absolute bottom-0 left-0 right-0 text-center">
          <div className="text-white text-sm font-semibold">
            {data.value.toFixed(1)} {data.unit}
          </div>
        </div>
      </div>
    );
  };

  const renderBarChart = () => {
    const percentage = getPercentage();
    
    return (
      <div className={`${getSizeClasses()} flex flex-col justify-end`}>
        <div className="flex-1 flex items-end">
          <div className="w-full bg-gray-700 rounded-t">
            <div
              className="bg-gradient-to-t from-gray-600 to-gray-500 rounded-t transition-all duration-500"
              style={{ height: `${percentage}%` }}
            />
          </div>
        </div>
        <div className="text-center mt-2">
          <div className="text-white font-bold text-sm">
            {data.value.toFixed(1)}
          </div>
          <div className="text-gray-400 text-xs">
            {data.unit}
          </div>
        </div>
      </div>
    );
  };

  const renderRadialChart = () => {
    const percentage = getPercentage();
    const angle = (percentage / 100) * 360;
    
    return (
      <div className={`${getSizeClasses()} relative`}>
        <div className="w-full h-full rounded-full border-4 border-gray-600 relative overflow-hidden">
          <div
            className="absolute inset-0 rounded-full transition-all duration-500"
            style={{
              background: `conic-gradient(${color} 0deg ${angle}deg, transparent ${angle}deg 360deg)`
            }}
          />
          <div className="absolute inset-2 bg-gray-800 rounded-full flex items-center justify-center">
            <div className="text-center">
              <div className="text-white font-bold text-sm">
                {data.value.toFixed(1)}
              </div>
              <div className="text-gray-400 text-xs">
                {data.unit}
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  };

  const renderVisualization = () => {
    switch (type) {
      case 'line': return renderLineChart();
      case 'bar': return renderBarChart();
      case 'radial': return renderRadialChart();
      default: return renderGauge();
    }
  };

  return (
    <div className="bg-gray-800 rounded-lg p-4 hover:shadow-lg transition-all duration-300">
      <div className="flex items-center justify-between mb-3">
        <h4 className="text-white font-semibold capitalize">
          {sensorId.replace(/([A-Z])/g, ' $1').trim()}
        </h4>
        <div className="text-xs text-gray-400">
          {new Date(data.timestamp).toLocaleTimeString()}
        </div>
      </div>
      
      <div className="flex justify-center">
        {renderVisualization()}
      </div>

      {showHistory && history.length > 1 && (
        <div className="mt-3 pt-3 border-t border-gray-700">
          <div className="text-xs text-gray-400 mb-2">Recent History</div>
          <div className="flex justify-between text-xs text-gray-300">
            <span>Min: {Math.min(...history.map(h => h.value)).toFixed(1)}</span>
            <span>Max: {Math.max(...history.map(h => h.value)).toFixed(1)}</span>
            <span>Avg: {(history.reduce((sum, h) => sum + h.value, 0) / history.length).toFixed(1)}</span>
          </div>
        </div>
      )}
    </div>
  );
};

export default SensorVisualization;
