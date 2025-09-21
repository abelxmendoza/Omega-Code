import React, { useState, useEffect } from 'react';

interface AnalyticsData {
  performance: {
    uptime: number;
    commandsExecuted: number;
    averageResponseTime: number;
    errorRate: number;
  };
  usage: {
    totalSessions: number;
    activeUsers: number;
    commandsPerHour: number;
    popularCommands: Array<{ command: string; count: number }>;
  };
  sensors: {
    ultrasonic: Array<{ timestamp: number; value: number }>;
    temperature: Array<{ timestamp: number; value: number }>;
    battery: Array<{ timestamp: number; value: number }>;
  };
  errors: Array<{
    timestamp: number;
    type: string;
    message: string;
    severity: 'low' | 'medium' | 'high';
  }>;
}

const AnalyticsDashboard: React.FC = () => {
  const [data, setData] = useState<AnalyticsData>({
    performance: {
      uptime: 0,
      commandsExecuted: 0,
      averageResponseTime: 0,
      errorRate: 0
    },
    usage: {
      totalSessions: 0,
      activeUsers: 0,
      commandsPerHour: 0,
      popularCommands: []
    },
    sensors: {
      ultrasonic: [],
      temperature: [],
      battery: []
    },
    errors: []
  });

  const [timeRange, setTimeRange] = useState<'1h' | '24h' | '7d' | '30d'>('24h');

  // Simulate analytics data
  useEffect(() => {
    const generateMockData = () => {
      const now = Date.now();
      const timeRangeMs = {
        '1h': 60 * 60 * 1000,
        '24h': 24 * 60 * 60 * 1000,
        '7d': 7 * 24 * 60 * 60 * 1000,
        '30d': 30 * 24 * 60 * 60 * 1000
      }[timeRange];

      setData({
        performance: {
          uptime: Math.floor(Math.random() * 100) + 95, // 95-100%
          commandsExecuted: Math.floor(Math.random() * 1000) + 500,
          averageResponseTime: Math.random() * 50 + 10, // 10-60ms
          errorRate: Math.random() * 2 // 0-2%
        },
        usage: {
          totalSessions: Math.floor(Math.random() * 100) + 50,
          activeUsers: Math.floor(Math.random() * 20) + 5,
          commandsPerHour: Math.floor(Math.random() * 100) + 30,
          popularCommands: [
            { command: 'forward', count: Math.floor(Math.random() * 100) + 50 },
            { command: 'stop', count: Math.floor(Math.random() * 80) + 40 },
            { command: 'left', count: Math.floor(Math.random() * 60) + 30 },
            { command: 'right', count: Math.floor(Math.random() * 60) + 30 },
            { command: 'backward', count: Math.floor(Math.random() * 40) + 20 }
          ].sort((a, b) => b.count - a.count)
        },
        sensors: {
          ultrasonic: Array.from({ length: 20 }, (_, i) => ({
            timestamp: now - (timeRangeMs / 20) * i,
            value: Math.random() * 100 + 20
          })),
          temperature: Array.from({ length: 20 }, (_, i) => ({
            timestamp: now - (timeRangeMs / 20) * i,
            value: Math.random() * 10 + 20
          })),
          battery: Array.from({ length: 20 }, (_, i) => ({
            timestamp: now - (timeRangeMs / 20) * i,
            value: Math.max(0, 100 - (i * 2) + Math.random() * 4)
          }))
        },
        errors: [
          {
            timestamp: now - 1000,
            type: 'WebSocket Connection',
            message: 'Connection timeout',
            severity: 'medium'
          },
          {
            timestamp: now - 5000,
            type: 'Sensor Reading',
            message: 'Ultrasonic sensor reading failed',
            severity: 'low'
          }
        ]
      });
    };

    generateMockData();
    const interval = setInterval(generateMockData, 5000);
    return () => clearInterval(interval);
  }, [timeRange]);

  const formatUptime = (uptime: number) => {
    const hours = Math.floor(uptime / 3600);
    const minutes = Math.floor((uptime % 3600) / 60);
    return `${hours}h ${minutes}m`;
  };

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case 'high': return 'text-red-400';
      case 'medium': return 'text-yellow-400';
      case 'low': return 'text-green-400';
      default: return 'text-gray-400';
    }
  };

  return (
    <div className="p-6 bg-gray-900 min-h-screen">
      <div className="max-w-7xl mx-auto">
        <h1 className="text-3xl font-bold text-white mb-8">Analytics Dashboard</h1>
        
        {/* Time Range Selector */}
        <div className="mb-6">
          <div className="flex gap-2">
            {(['1h', '24h', '7d', '30d'] as const).map(range => (
              <button
                key={range}
                className={`px-4 py-2 rounded font-semibold transition-all duration-200 hover:scale-105 ${
                  timeRange === range
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                }`}
                onClick={() => setTimeRange(range)}
              >
                {range}
              </button>
            ))}
          </div>
        </div>

        {/* Performance Metrics */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-2">Uptime</h3>
            <div className="text-3xl font-bold text-green-400">
              {data.performance.uptime.toFixed(1)}%
            </div>
            <div className="text-gray-400 text-sm">
              {formatUptime(data.performance.uptime * 3600 / 100)}
            </div>
          </div>

          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-2">Commands Executed</h3>
            <div className="text-3xl font-bold text-blue-400">
              {data.performance.commandsExecuted.toLocaleString()}
            </div>
            <div className="text-gray-400 text-sm">
              Last {timeRange}
            </div>
          </div>

          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-2">Avg Response Time</h3>
            <div className="text-3xl font-bold text-yellow-400">
              {data.performance.averageResponseTime.toFixed(1)}ms
            </div>
            <div className="text-gray-400 text-sm">
              WebSocket commands
            </div>
          </div>

          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-2">Error Rate</h3>
            <div className="text-3xl font-bold text-red-400">
              {data.performance.errorRate.toFixed(2)}%
            </div>
            <div className="text-gray-400 text-sm">
              System errors
            </div>
          </div>
        </div>

        {/* Usage Statistics */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-8">
          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-4">Usage Statistics</h3>
            <div className="space-y-4">
              <div className="flex justify-between">
                <span className="text-gray-300">Total Sessions</span>
                <span className="text-white font-semibold">{data.usage.totalSessions}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-300">Active Users</span>
                <span className="text-white font-semibold">{data.usage.activeUsers}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-300">Commands/Hour</span>
                <span className="text-white font-semibold">{data.usage.commandsPerHour}</span>
              </div>
            </div>
          </div>

          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-4">Popular Commands</h3>
            <div className="space-y-2">
              {data.usage.popularCommands.map((cmd, index) => (
                <div key={cmd.command} className="flex justify-between items-center">
                  <span className="text-gray-300 capitalize">{cmd.command}</span>
                  <div className="flex items-center gap-2">
                    <div className="w-20 bg-gray-700 rounded-full h-2">
                      <div 
                        className="bg-blue-500 h-2 rounded-full transition-all duration-300"
                        style={{ width: `${(cmd.count / data.usage.popularCommands[0].count) * 100}%` }}
                      />
                    </div>
                    <span className="text-white font-semibold text-sm w-8 text-right">
                      {cmd.count}
                    </span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Sensor Data Charts */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8">
          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-4">Ultrasonic Sensor</h3>
            <div className="h-32 flex items-end gap-1">
              {data.sensors.ultrasonic.map((point, index) => (
                <div
                  key={index}
                  className="bg-green-500 flex-1 rounded-t transition-all duration-300"
                  style={{ height: `${(point.value / 200) * 100}%` }}
                />
              ))}
            </div>
            <div className="text-gray-400 text-sm mt-2">
              Distance (cm) - Last 20 readings
            </div>
          </div>

          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-4">Temperature</h3>
            <div className="h-32 flex items-end gap-1">
              {data.sensors.temperature.map((point, index) => (
                <div
                  key={index}
                  className="bg-yellow-500 flex-1 rounded-t transition-all duration-300"
                  style={{ height: `${((point.value - 15) / 15) * 100}%` }}
                />
              ))}
            </div>
            <div className="text-gray-400 text-sm mt-2">
              Temperature (°C) - Last 20 readings
            </div>
          </div>

          <div className="bg-gray-800 p-6 rounded-lg">
            <h3 className="text-white font-semibold mb-4">Battery Level</h3>
            <div className="h-32 flex items-end gap-1">
              {data.sensors.battery.map((point, index) => (
                <div
                  key={index}
                  className={`flex-1 rounded-t transition-all duration-300 ${
                    point.value > 50 ? 'bg-green-500' :
                    point.value > 20 ? 'bg-yellow-500' : 'bg-red-500'
                  }`}
                  style={{ height: `${point.value}%` }}
                />
              ))}
            </div>
            <div className="text-gray-400 text-sm mt-2">
              Battery (%) - Last 20 readings
            </div>
          </div>
        </div>

        {/* Error Log */}
        <div className="bg-gray-800 p-6 rounded-lg">
          <h3 className="text-white font-semibold mb-4">Recent Errors</h3>
          <div className="space-y-3">
            {data.errors.length > 0 ? (
              data.errors.map((error, index) => (
                <div key={index} className="flex justify-between items-start p-3 bg-gray-700 rounded">
                  <div className="flex-1">
                    <div className="flex items-center gap-2 mb-1">
                      <span className={`font-semibold ${getSeverityColor(error.severity)}`}>
                        {error.severity.toUpperCase()}
                      </span>
                      <span className="text-gray-400 text-sm">
                        {new Date(error.timestamp).toLocaleString()}
                      </span>
                    </div>
                    <div className="text-white font-semibold">{error.type}</div>
                    <div className="text-gray-300 text-sm">{error.message}</div>
                  </div>
                </div>
              ))
            ) : (
              <div className="text-gray-400 text-center py-8">
                No errors in the selected time range
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default AnalyticsDashboard;
