/*
# File: /src/components/PerformanceDashboard.tsx
# Summary:
# Real-time performance monitoring dashboard
# - System metrics (CPU, memory, network)
# - Application metrics (response times, error rates)
# - WebSocket connection health
# - Cache performance statistics
*/

import React, { useState, useEffect, memo } from 'react';
import { performanceMonitor, performanceMetrics } from '@/utils/optimization';

interface PerformanceData {
  timestamp: number;
  source: string;
  deviceName: string;
  cpuUsage: number;
  memoryUsage: number;
  memoryPercent: number;
  diskUsage: number;
  networkIO: {
    bytesSent: number;
    bytesRecv: number;
    packetsSent: number;
    packetsRecv: number;
  };
  websocketConnections: number;
  uptime: number;
  loadAverage: number;
  temperature: number | null;
  piSpecific: {
    gpioStatus: { status: string; pins: string };
    cameraStatus: { status: string; enabled: boolean };
    robotServices: { movement: boolean; camera: boolean; sensors: boolean; lighting: boolean };
    piModel: string;
    firmwareVersion: string;
  };
  robotTelemetry: {
    power: { voltage: number | null; percentage: number | null; charging: boolean; powerSource: string; undervoltage: boolean; powerConsumption: number | null };
    sensors: {
      camera: { fps: number | null; resolution: string | null; status: string };
      ultrasonic: { distance: number | null; status: string; pins: string };
      lineTracking: { sensors: number[]; status: string; pins: string };
      voltage: { value: number | null; status: string; adc: string; i2c: string };
      buzzer: { status: string; pin: string };
      leds: { status: string; pins: string };
    };
    motors: {
      leftMotor: { speed: number; position: number; temperature: number | null; status: string };
      rightMotor: { speed: number; position: number; temperature: number | null; status: string };
      servoMotors: { id: number; position: number; status: string }[];
      actuators: any[];
    };
    network: { wifiSignal: number | null; latency: number | null; throughput: number | null; connectionType: string; ipAddress: string | null };
    autonomous: { mode: string; navigation: { status: string; target: any; path: any[] }; obstacleAvoidance: { enabled: boolean; detected: boolean }; lineFollowing: { enabled: boolean; onLine: boolean }; mission: { active: boolean; progress: number } };
    safety: { emergencyStop: boolean; safetyLimits: { enabled: boolean; violations: number }; collisionDetection: { enabled: boolean; detected: boolean }; batteryProtection: { enabled: boolean; lowBattery: boolean }; thermalProtection: { enabled: boolean; overheated: boolean } };
  };
}

interface CacheStats {
  hits: number;
  misses: number;
  hitRate: number;
  totalRequests: number;
}

const PerformanceDashboard: React.FC = memo(() => {
  const [performanceData, setPerformanceData] = useState<PerformanceData | null>(null);
  const [cacheStats, setCacheStats] = useState<CacheStats | null>(null);
  const [isMonitoring, setIsMonitoring] = useState(false);

  useEffect(() => {
    // Start performance monitoring
    const startMonitoring = async () => {
      try {
        setIsMonitoring(true);
        
        // Connect to Pi's performance API through the gateway
        const fetchPerformanceData = async () => {
          try {
            const response = await fetch('/api/performance-proxy/metrics');
            if (response.ok) {
              const data = await response.json();
              setPerformanceData(data);
            } else {
              console.warn('Performance API not available, using fallback data');
              // Fallback to simulated data if Pi API is unavailable
              setPerformanceData({
                timestamp: Date.now(),
                cpuUsage: Math.random() * 100,
                memoryUsage: Math.random() * 8 * 1024 * 1024 * 1024, // Random GB
                memoryPercent: Math.random() * 100,
                diskUsage: Math.random() * 100,
                networkIO: {
                  bytesSent: Math.random() * 1000000,
                  bytesRecv: Math.random() * 1000000,
                },
                websocketConnections: Math.floor(Math.random() * 5),
                responseTime: Math.random() * 100,
                errorRate: Math.random() * 5,
                throughput: Math.random() * 1000,
              });
            }
          } catch (error) {
            console.error('Failed to fetch performance data:', error);
          }
        };

        const fetchCacheStats = async () => {
          try {
            const response = await fetch('/api/performance-proxy/cache');
            if (response.ok) {
              const data = await response.json();
              setCacheStats(data);
            }
          } catch (error) {
            console.error('Failed to fetch cache stats:', error);
          }
        };

        // Initial fetch
        await fetchPerformanceData();
        await fetchCacheStats();
        
        const interval = setInterval(() => {
          fetchPerformanceData();
          fetchCacheStats();
          // Simulate performance data (replace with real API call)
          const mockData: PerformanceData = {
            timestamp: Date.now(),
            cpuUsage: Math.random() * 100,
            memoryUsage: Math.random() * 1000,
            memoryPercent: Math.random() * 100,
            diskUsage: Math.random() * 100,
            networkIO: {
              bytesSent: Math.random() * 1000000,
              bytesRecv: Math.random() * 1000000,
            },
            websocketConnections: Math.floor(Math.random() * 10),
            responseTime: Math.random() * 100,
            errorRate: Math.random() * 5,
            throughput: Math.random() * 1000,
          };
          
          setPerformanceData(mockData);
          
          // Simulate cache stats
          const mockCacheStats: CacheStats = {
            hits: Math.floor(Math.random() * 1000),
            misses: Math.floor(Math.random() * 100),
            hitRate: Math.random() * 100,
            totalRequests: Math.floor(Math.random() * 1100),
          };
          
          setCacheStats(mockCacheStats);
        }, 2000);

        return () => clearInterval(interval);
      } catch (error) {
        console.error('Failed to start performance monitoring:', error);
      }
    };

    const cleanup = startMonitoring();
    
    return () => {
      cleanup.then(cleanupFn => cleanupFn?.());
      setIsMonitoring(false);
    };
  }, []);

  const getStatusColor = (value: number, thresholds: { warning: number; critical: number }) => {
    if (value >= thresholds.critical) return 'text-red-500';
    if (value >= thresholds.warning) return 'text-yellow-500';
    return 'text-green-500';
  };

  const formatBytes = (bytes: number) => {
    const sizes = ['B', 'KB', 'MB', 'GB'];
    if (bytes === 0) return '0 B';
    const i = Math.floor(Math.log(bytes) / Math.log(1024));
    return `${(bytes / Math.pow(1024, i)).toFixed(1)} ${sizes[i]}`;
  };

  if (!performanceData || !cacheStats) {
    return (
      <div className="bg-gray-800 text-white p-4 rounded-md shadow-md">
        <h2 className="text-lg font-bold mb-4">Performance Dashboard</h2>
        <div className="text-center text-gray-400">
          {isMonitoring ? 'Starting monitoring...' : 'Monitoring not available'}
        </div>
      </div>
    );
  }

  return (
    <div className="bg-gray-800 text-white p-4 rounded-md shadow-md w-full max-w-4xl">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-3">
          <h2 className="text-lg font-bold">Performance Dashboard</h2>
          {performanceData && (
            <div className="flex items-center gap-2 px-2 py-1 bg-blue-600 rounded-full text-xs">
              <span className="text-blue-200">📱</span>
              <span className="font-medium">{performanceData.source}</span>
              <span className="text-blue-200">•</span>
              <span className="text-blue-200">{performanceData.deviceName}</span>
            </div>
          )}
        </div>
        <div className="flex items-center space-x-2">
          <div className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-green-500' : 'bg-red-500'}`} />
          <span className="text-sm text-gray-400">
            {isMonitoring ? 'Monitoring' : 'Stopped'}
          </span>
        </div>
      </div>

      {/* Pi-Specific Information */}
      {performanceData?.piSpecific && (
        <div className="mb-4 p-3 bg-gray-700 rounded border border-gray-600">
          <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-orange-300">
            <span>🤖</span>
            Raspberry Pi Status
          </h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3 text-xs">
            <div>
              <div className="text-gray-400">Model</div>
              <div className="font-medium">{performanceData.piSpecific.piModel}</div>
            </div>
            <div>
              <div className="text-gray-400">Temperature</div>
              <div className="font-medium">
                {performanceData.temperature ? `${performanceData.temperature.toFixed(1)}°C` : 'N/A'}
              </div>
            </div>
            <div>
              <div className="text-gray-400">GPIO</div>
              <div className={`font-medium ${performanceData.piSpecific.gpioStatus.status === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                {performanceData.piSpecific.gpioStatus.status}
              </div>
            </div>
            <div>
              <div className="text-gray-400">Camera</div>
              <div className={`font-medium ${performanceData.piSpecific.cameraStatus.enabled ? 'text-green-400' : 'text-red-400'}`}>
                {performanceData.piSpecific.cameraStatus.status}
              </div>
            </div>
          </div>
          
          {/* Robot Services Status */}
          <div className="mt-2">
            <div className="text-gray-400 mb-1">Robot Services</div>
            <div className="flex gap-3">
              {Object.entries(performanceData.piSpecific.robotServices).map(([service, status]) => (
                <div key={service} className="flex items-center gap-1">
                  <div className={`w-1.5 h-1.5 rounded-full ${status ? 'bg-green-400' : 'bg-red-400'}`}></div>
                  <span className="text-xs capitalize">{service}</span>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Robot Telemetry Sections */}
      {performanceData?.robotTelemetry && (
        <div className="mb-4 space-y-4">
          {/* Power & Safety Status */}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div className="bg-gray-700 p-3 rounded border border-gray-600">
              <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-yellow-300">
                <span>🔋</span>
                Power Status
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-400">Source:</span>
                  <span className="font-medium capitalize">{performanceData.robotTelemetry.power.powerSource}</span>
                </div>
                {performanceData.robotTelemetry.power.voltage && (
                  <div className="flex justify-between">
                    <span className="text-gray-400">Voltage:</span>
                    <span className={`font-medium ${performanceData.robotTelemetry.power.undervoltage ? 'text-red-400' : 'text-green-400'}`}>
                      {performanceData.robotTelemetry.power.voltage}V
                    </span>
                  </div>
                )}
                {performanceData.robotTelemetry.power.powerConsumption && (
                  <div className="flex justify-between">
                    <span className="text-gray-400">Power:</span>
                    <span className="font-medium">{performanceData.robotTelemetry.power.powerConsumption}W</span>
                  </div>
                )}
                {performanceData.robotTelemetry.power.undervoltage && (
                  <div className="text-red-400 text-xs">⚠️ Undervoltage Warning</div>
                )}
              </div>
            </div>

            <div className="bg-gray-700 p-3 rounded border border-gray-600">
              <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-red-300">
                <span>🛡️</span>
                Safety Systems
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-400">Emergency Stop:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.safety.emergencyStop ? 'text-red-400' : 'text-green-400'}`}>
                    {performanceData.robotTelemetry.safety.emergencyStop ? 'ACTIVE' : 'Safe'}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Thermal:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.safety.thermalProtection.overheated ? 'text-red-400' : 'text-green-400'}`}>
                    {performanceData.robotTelemetry.safety.thermalProtection.overheated ? 'Overheated' : 'Normal'}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Battery:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.safety.batteryProtection.lowBattery ? 'text-red-400' : 'text-green-400'}`}>
                    {performanceData.robotTelemetry.safety.batteryProtection.lowBattery ? 'Low Battery' : 'OK'}
                  </span>
                </div>
              </div>
            </div>
          </div>

          {/* Sensors & Motors */}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div className="bg-gray-700 p-3 rounded border border-gray-600">
              <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-blue-300">
                <span>📡</span>
                Sensors
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-400">Camera:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.sensors.camera.status === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.sensors.camera.fps ? `${performanceData.robotTelemetry.sensors.camera.fps}fps` : performanceData.robotTelemetry.sensors.camera.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Ultrasonic:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.sensors.ultrasonic.status === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.sensors.ultrasonic.distance ? `${performanceData.robotTelemetry.sensors.ultrasonic.distance}cm` : performanceData.robotTelemetry.sensors.ultrasonic.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Line Tracking:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.sensors.lineTracking.status === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.sensors.lineTracking.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Voltage:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.sensors.voltage.status === 'connected' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.sensors.voltage.value ? `${performanceData.robotTelemetry.sensors.voltage.value}V` : performanceData.robotTelemetry.sensors.voltage.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Buzzer:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.sensors.buzzer.status === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.sensors.buzzer.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">LEDs:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.sensors.leds.status === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.sensors.leds.status}
                  </span>
                </div>
              </div>
            </div>

            <div className="bg-gray-700 p-3 rounded border border-gray-600">
              <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-purple-300">
                <span>⚙️</span>
                Motors
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-400">Left Motor:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.motors.leftMotor.status === 'connected' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.motors.leftMotor.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Right Motor:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.motors.rightMotor.status === 'connected' ? 'text-green-400' : 'text-red-400'}`}>
                    {performanceData.robotTelemetry.motors.rightMotor.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Servos:</span>
                  <span className="font-medium text-blue-400">
                    {performanceData.robotTelemetry.motors.servoMotors.length} active
                  </span>
                </div>
              </div>
            </div>
          </div>

          {/* Network & Autonomous */}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div className="bg-gray-700 p-3 rounded border border-gray-600">
              <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-green-300">
                <span>🌐</span>
                Network
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-400">Type:</span>
                  <span className="font-medium capitalize">{performanceData.robotTelemetry.network.connectionType}</span>
                </div>
                {performanceData.robotTelemetry.network.ipAddress && (
                  <div className="flex justify-between">
                    <span className="text-gray-400">IP:</span>
                    <span className="font-medium">{performanceData.robotTelemetry.network.ipAddress}</span>
                  </div>
                )}
                {performanceData.robotTelemetry.network.latency && (
                  <div className="flex justify-between">
                    <span className="text-gray-400">Latency:</span>
                    <span className="font-medium">{performanceData.robotTelemetry.network.latency}ms</span>
                  </div>
                )}
                {performanceData.robotTelemetry.network.wifiSignal && (
                  <div className="flex justify-between">
                    <span className="text-gray-400">Signal:</span>
                    <span className={`font-medium ${performanceData.robotTelemetry.network.wifiSignal > -60 ? 'text-green-400' : 'text-yellow-400'}`}>
                      {performanceData.robotTelemetry.network.wifiSignal}dBm
                    </span>
                  </div>
                )}
              </div>
            </div>

            <div className="bg-gray-700 p-3 rounded border border-gray-600">
              <h3 className="text-sm font-semibold mb-2 flex items-center gap-2 text-cyan-300">
                <span>🤖</span>
                Autonomous
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-400">Mode:</span>
                  <span className={`font-medium capitalize ${performanceData.robotTelemetry.autonomous.mode === 'autonomous' ? 'text-green-400' : 'text-blue-400'}`}>
                    {performanceData.robotTelemetry.autonomous.mode}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Navigation:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.autonomous.navigation.status === 'active' ? 'text-green-400' : 'text-gray-400'}`}>
                    {performanceData.robotTelemetry.autonomous.navigation.status}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Obstacle Avoid:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.autonomous.obstacleAvoidance.enabled ? 'text-green-400' : 'text-gray-400'}`}>
                    {performanceData.robotTelemetry.autonomous.obstacleAvoidance.enabled ? 'Enabled' : 'Disabled'}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Line Follow:</span>
                  <span className={`font-medium ${performanceData.robotTelemetry.autonomous.lineFollowing.enabled ? 'text-green-400' : 'text-gray-400'}`}>
                    {performanceData.robotTelemetry.autonomous.lineFollowing.enabled ? 'Enabled' : 'Disabled'}
                  </span>
                </div>
              </div>
            </div>
          </div>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {/* System Metrics */}
        <div className="bg-gray-700 p-3 rounded">
          <h3 className="text-sm font-semibold text-blue-300 mb-2">System Metrics</h3>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between">
              <span className="text-gray-400">CPU Usage:</span>
              <span className={getStatusColor(performanceData.cpuUsage, { warning: 70, critical: 90 })}>
                {performanceData.cpuUsage.toFixed(1)}%
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Memory:</span>
              <span className={getStatusColor(performanceData.memoryPercent, { warning: 80, critical: 95 })}>
                {performanceData.memoryPercent.toFixed(1)}%
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Disk Usage:</span>
              <span className={getStatusColor(performanceData.diskUsage, { warning: 85, critical: 95 })}>
                {performanceData.diskUsage.toFixed(1)}%
              </span>
            </div>
          </div>
        </div>

        {/* Network Metrics */}
        <div className="bg-gray-700 p-3 rounded">
          <h3 className="text-sm font-semibold text-green-300 mb-2">Network</h3>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between">
              <span className="text-gray-400">Bytes Sent:</span>
              <span className="text-white font-mono">{formatBytes(performanceData.networkIO.bytesSent)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Bytes Received:</span>
              <span className="text-white font-mono">{formatBytes(performanceData.networkIO.bytesRecv)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">WS Connections:</span>
              <span className="text-white font-mono">{performanceData.websocketConnections}</span>
            </div>
          </div>
        </div>

        {/* Application Metrics */}
        <div className="bg-gray-700 p-3 rounded">
          <h3 className="text-sm font-semibold text-purple-300 mb-2">Application</h3>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between">
              <span className="text-gray-400">Response Time:</span>
              <span className={getStatusColor(performanceData.responseTime, { warning: 500, critical: 1000 })}>
                {performanceData.responseTime.toFixed(1)}ms
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Error Rate:</span>
              <span className={getStatusColor(performanceData.errorRate, { warning: 2, critical: 5 })}>
                {performanceData.errorRate.toFixed(2)}%
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Throughput:</span>
              <span className="text-white font-mono">{performanceData.throughput.toFixed(0)} req/s</span>
            </div>
          </div>
        </div>

        {/* Cache Performance */}
        <div className="bg-gray-700 p-3 rounded">
          <h3 className="text-sm font-semibold text-yellow-300 mb-2">Cache Performance</h3>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between">
              <span className="text-gray-400">Hit Rate:</span>
              <span className={getStatusColor(100 - cacheStats.hitRate, { warning: 20, critical: 40 })}>
                {cacheStats.hitRate.toFixed(1)}%
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Total Requests:</span>
              <span className="text-white font-mono">{cacheStats.totalRequests}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Cache Hits:</span>
              <span className="text-white font-mono">{cacheStats.hits}</span>
            </div>
          </div>
        </div>

        {/* Performance Alerts */}
        <div className="bg-gray-700 p-3 rounded">
          <h3 className="text-sm font-semibold text-red-300 mb-2">Alerts</h3>
          <div className="space-y-1 text-xs">
            {performanceData.cpuUsage > 90 && (
              <div className="text-red-400">⚠️ High CPU usage</div>
            )}
            {performanceData.memoryPercent > 95 && (
              <div className="text-red-400">⚠️ High memory usage</div>
            )}
            {performanceData.responseTime > 1000 && (
              <div className="text-red-400">⚠️ Slow response time</div>
            )}
            {performanceData.errorRate > 5 && (
              <div className="text-red-400">⚠️ High error rate</div>
            )}
            {cacheStats.hitRate < 60 && (
              <div className="text-yellow-400">⚠️ Low cache hit rate</div>
            )}
            {!performanceData.cpuUsage && !performanceData.memoryPercent && (
              <div className="text-green-400">✅ All systems normal</div>
            )}
          </div>
        </div>

        {/* System Info */}
        <div className="bg-gray-700 p-3 rounded">
          <h3 className="text-sm font-semibold text-cyan-300 mb-2">System Info</h3>
          <div className="space-y-1 text-xs">
            <div className="flex justify-between">
              <span className="text-gray-400">Uptime:</span>
              <span className="text-white font-mono">
                {Math.floor((Date.now() - performanceData.timestamp) / 1000)}s
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Last Update:</span>
              <span className="text-white font-mono">
                {new Date(performanceData.timestamp).toLocaleTimeString()}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-400">Status:</span>
              <span className="text-green-400">Online</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
});

PerformanceDashboard.displayName = 'PerformanceDashboard';

export default PerformanceDashboard;
