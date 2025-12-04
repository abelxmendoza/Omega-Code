/**
 * ROS 2 Dashboard Page
 * 
 * Comprehensive ROS 2 management and monitoring interface with built-in debugging tools.
 * Use browser DevTools (F12) to monitor network requests, WebSocket connections, and console logs.
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import Head from 'next/head';
import Link from 'next/link';
import { ROSManagementPanel, TelemetryVisualization } from '@/components/ros';
import { getROSStatus, controlROSContainer, listROSTopics } from '@/utils/rosApi';
import { buildGatewayUrl } from '@/config/gateway';

// Debug mode - logs to console and DevTools
const DEBUG = process.env.NEXT_PUBLIC_ROS_DEBUG === '1' || typeof window !== 'undefined' && window.localStorage.getItem('ros_debug') === 'true';

// Debug logger that works with browser DevTools
const debugLog = {
  info: (...args: any[]) => {
    if (DEBUG) {
      console.log('%c[ROS Debug]', 'color: #00ff00; font-weight: bold', ...args);
    }
  },
  error: (...args: any[]) => {
    console.error('%c[ROS Error]', 'color: #ff0000; font-weight: bold', ...args);
  },
  warn: (...args: any[]) => {
    console.warn('%c[ROS Warning]', 'color: #ffaa00; font-weight: bold', ...args);
  },
  network: (method: string, url: string, data?: any) => {
    if (DEBUG) {
      console.log('%c[ROS Network]', 'color: #00aaff; font-weight: bold', method, url, data || '');
    }
  },
  ws: (event: string, data?: any) => {
    if (DEBUG) {
      console.log('%c[ROS WebSocket]', 'color: #ff00ff; font-weight: bold', event, data || '');
    }
  }
};

interface DebugInfo {
  timestamp: number;
  type: 'api' | 'websocket' | 'error' | 'status';
  message: string;
  data?: any;
}

export default function ROSDashboard() {
  const [status, setStatus] = useState<any>(null);
  const [topics, setTopics] = useState<string[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [debugLogs, setDebugLogs] = useState<DebugInfo[]>([]);
  const [wsConnected, setWsConnected] = useState(false);
  const [wsReconnectAttempts, setWsReconnectAttempts] = useState(0);
  const wsRef = useRef<WebSocket | null>(null);
  const [autoRefresh, setAutoRefresh] = useState(true);
  const [refreshInterval, setRefreshInterval] = useState(5000);

  // Add debug log entry
  const addDebugLog = useCallback((type: DebugInfo['type'], message: string, data?: any) => {
    const entry: DebugInfo = {
      timestamp: Date.now(),
      type,
      message,
      data: data ? JSON.parse(JSON.stringify(data)) : undefined
    };
    setDebugLogs(prev => [entry, ...prev].slice(0, 200)); // Keep last 200 logs
    
    // Also log to console for DevTools
    switch (type) {
      case 'error':
        debugLog.error(message, data);
        break;
      case 'api':
        debugLog.network('API', message, data);
        break;
      case 'websocket':
        debugLog.ws(message, data);
        break;
      default:
        debugLog.info(message, data);
    }
  }, []);

  // Fetch ROS status
  const fetchStatus = useCallback(async () => {
    try {
      addDebugLog('api', 'Fetching ROS status...');
      const url = await buildGatewayUrl('/api/ros/status');
      debugLog.network('GET', url);
      
      const startTime = performance.now();
      const response = await fetch(url);
      const duration = performance.now() - startTime;
      
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      addDebugLog('status', `Status fetched in ${duration.toFixed(2)}ms`, { containers: data.containers?.length, topics: data.topics?.length });
      
      setStatus(data);
      setTopics(data.topics || []);
      setError(null);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to fetch status';
      addDebugLog('error', 'Failed to fetch status', { error: errorMsg });
      setError(errorMsg);
    } finally {
      setLoading(false);
    }
  }, [addDebugLog]);

  // Fetch topics
  const fetchTopics = useCallback(async () => {
    try {
      addDebugLog('api', 'Fetching ROS topics...');
      const url = await buildGatewayUrl('/api/ros/topics');
      debugLog.network('GET', url);
      
      const data = await listROSTopics();
      addDebugLog('status', `Found ${data.length} topics`, data);
      setTopics(data);
    } catch (err) {
      addDebugLog('error', 'Failed to fetch topics', { error: err });
    }
  }, [addDebugLog]);

  // Control ROS containers
  const handleControl = async (action: 'start' | 'stop' | 'restart', service?: string) => {
    try {
      addDebugLog('api', `${action} container${service ? `: ${service}` : 's'}...`);
      const url = await buildGatewayUrl('/api/ros/control');
      debugLog.network('POST', url, { action, service });
      
      const result = await controlROSContainer({ action, service });
      
      if (result.success) {
        addDebugLog('status', `Successfully ${action}ed ${service || 'all containers'}`, result);
        setTimeout(() => {
          fetchStatus();
          fetchTopics();
        }, 1000);
      } else {
        throw new Error(result.error || 'Unknown error');
      }
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Action failed';
      addDebugLog('error', `Failed to ${action} container`, { error: errorMsg });
      setError(errorMsg);
    }
  };

  // WebSocket connection for telemetry
  useEffect(() => {
    let reconnectTimeout: NodeJS.Timeout | null = null;
    let reconnectAttempts = 0;
    const maxReconnectAttempts = 10;

    const connectWebSocket = async () => {
      try {
        if (wsRef.current?.readyState === WebSocket.OPEN) {
          return; // Already connected
        }

        const wsUrl = await buildGatewayUrl('/ws/ros/telemetry');
        const protocol = wsUrl.startsWith('https') ? 'wss' : 'ws';
        const url = wsUrl.replace(/^https?/, protocol);
        
        addDebugLog('websocket', `Connecting to ${url}...`);
        debugLog.ws('connect', url);
        
        wsRef.current = new WebSocket(url);

        wsRef.current.onopen = () => {
          addDebugLog('websocket', 'WebSocket connected');
          debugLog.ws('open', 'Connected');
          setWsConnected(true);
          setWsReconnectAttempts(0);
          reconnectAttempts = 0;
        };

        wsRef.current.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            addDebugLog('websocket', 'Message received', data);
            debugLog.ws('message', data);
          } catch {
            addDebugLog('websocket', 'Message received (raw)', { data: event.data });
            debugLog.ws('message', event.data);
          }
        };

        wsRef.current.onerror = (error) => {
          addDebugLog('error', 'WebSocket error', { error });
          debugLog.error('WebSocket error', error);
          setWsConnected(false);
        };

        wsRef.current.onclose = (event) => {
          addDebugLog('websocket', `WebSocket closed (code: ${event.code}, reason: ${event.reason || 'none'})`);
          debugLog.ws('close', { code: event.code, reason: event.reason });
          setWsConnected(false);
          
          // Attempt to reconnect
          if (reconnectAttempts < maxReconnectAttempts) {
            reconnectAttempts++;
            setWsReconnectAttempts(reconnectAttempts);
            const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 10000);
            addDebugLog('websocket', `Reconnecting in ${delay}ms (attempt ${reconnectAttempts}/${maxReconnectAttempts})...`);
            
            reconnectTimeout = setTimeout(() => {
              connectWebSocket();
            }, delay);
          } else {
            addDebugLog('error', 'Max reconnection attempts reached');
          }
        };
      } catch (err) {
        addDebugLog('error', 'Failed to create WebSocket', { error: err });
      }
    };

    connectWebSocket();

    return () => {
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, [addDebugLog]);

  // Auto-refresh status
  useEffect(() => {
    if (!autoRefresh) return;

    fetchStatus();
    const interval = setInterval(() => {
      fetchStatus();
      if (topics.length === 0) {
        fetchTopics();
      }
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [autoRefresh, refreshInterval, fetchStatus, fetchTopics, topics.length]);

  // Initial load
  useEffect(() => {
    fetchStatus();
    fetchTopics();
  }, []);

  // Format timestamp for display
  const formatTimestamp = (ts: number) => {
    return new Date(ts).toLocaleTimeString();
  };

  // Filter debug logs by type
  const filteredLogs = debugLogs; // Can add filtering UI later

  return (
    <>
      <Head>
        <title>ROS 2 Infrastructure Dashboard - Omega Robot</title>
        <meta name="description" content="ROS 2 Docker Management and Telemetry Dashboard" />
      </Head>

      <div className="min-h-screen bg-gradient-to-br from-gray-900 via-gray-800 to-gray-900 text-white p-4">
        <div className="max-w-7xl mx-auto">
          {/* Header */}
          <div className="mb-6">
            <div className="flex items-center justify-between mb-2">
              <div>
                <h1 className="text-3xl font-bold mb-2">ROS 2 Infrastructure Dashboard</h1>
                <p className="text-gray-400">Manage Docker containers, monitor topics, and view telemetry logs</p>
              </div>
              <Link 
                href="/" 
                className="text-sm text-blue-400 hover:text-blue-300 underline flex items-center gap-1 px-3 py-2 bg-gray-800 rounded hover:bg-gray-700"
              >
                ← Autonomy Settings
              </Link>
            </div>
            <div className="mt-2 p-3 bg-blue-900/20 border border-blue-500/50 rounded text-sm">
              <strong className="text-blue-300">💡 Note:</strong> This dashboard manages ROS <strong>infrastructure</strong> (containers, topics, logs). 
              To configure which ROS <strong>features</strong> are used during autonomy, visit the <Link href="/" className="text-blue-400 hover:text-blue-300 underline font-semibold">Autonomy Settings</Link>.
            </div>
            <div className="mt-4 flex items-center gap-4 text-sm">
              <label className="flex items-center gap-2">
                <input
                  type="checkbox"
                  checked={autoRefresh}
                  onChange={(e) => setAutoRefresh(e.target.checked)}
                  className="rounded"
                />
                Auto-refresh ({refreshInterval}ms)
              </label>
              <label className="flex items-center gap-2">
                <span>Interval:</span>
                <select
                  value={refreshInterval}
                  onChange={(e) => setRefreshInterval(Number(e.target.value))}
                  className="bg-gray-700 text-white rounded px-2 py-1"
                >
                  <option value={1000}>1s</option>
                  <option value={2000}>2s</option>
                  <option value={5000}>5s</option>
                  <option value={10000}>10s</option>
                </select>
              </label>
              <button
                onClick={() => {
                  setDebugLogs([]);
                  console.clear();
                }}
                className="px-3 py-1 bg-gray-700 hover:bg-gray-600 rounded text-sm"
              >
                Clear Debug Logs
              </button>
              <div className="flex items-center gap-2">
                <div className={`w-2 h-2 rounded-full ${wsConnected ? 'bg-green-500' : 'bg-red-500'}`} />
                <span>WebSocket: {wsConnected ? 'Connected' : 'Disconnected'}</span>
                {wsReconnectAttempts > 0 && (
                  <span className="text-yellow-400">(Reconnect: {wsReconnectAttempts})</span>
                )}
              </div>
            </div>
          </div>

          {/* Error Banner */}
          {error && (
            <div className="mb-4 p-4 bg-red-900/50 border border-red-500 rounded-lg">
              <div className="flex items-center justify-between">
                <div>
                  <strong className="text-red-200">Error:</strong>
                  <span className="ml-2">{error}</span>
                </div>
                <button
                  onClick={() => setError(null)}
                  className="text-red-300 hover:text-red-100"
                >
                  ×
                </button>
              </div>
            </div>
          )}

          {/* Main Content Grid */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
            {/* ROS Management Panel */}
            <div>
              <ROSManagementPanel className="mb-4" />
            </div>

            {/* Telemetry Visualization */}
            <div>
              <TelemetryVisualization className="mb-4" />
            </div>
          </div>

          {/* Debug Panel */}
          <div className="mb-6">
            <div className="bg-gray-800 rounded-lg p-4">
              <div className="flex items-center justify-between mb-4">
                <h2 className="text-xl font-semibold">Debug Logs</h2>
                <div className="flex items-center gap-2 text-sm">
                  <span className="text-gray-400">Open DevTools (F12) to view detailed logs</span>
                  <span className="text-gray-500">|</span>
                  <span className="text-gray-400">{filteredLogs.length} entries</span>
                </div>
              </div>
              
              <div className="bg-black rounded p-3 font-mono text-xs max-h-64 overflow-y-auto">
                {filteredLogs.length === 0 ? (
                  <div className="text-gray-500">No debug logs yet. Interact with ROS services to see logs.</div>
                ) : (
                  filteredLogs.map((log, idx) => (
                    <div key={idx} className="mb-1 border-b border-gray-800 pb-1">
                      <span className="text-gray-500">{formatTimestamp(log.timestamp)}</span>
                      <span className={`ml-2 ${
                        log.type === 'error' ? 'text-red-400' :
                        log.type === 'websocket' ? 'text-purple-400' :
                        log.type === 'api' ? 'text-blue-400' :
                        'text-green-400'
                      }`}>
                        [{log.type.toUpperCase()}]
                      </span>
                      <span className="ml-2 text-gray-300">{log.message}</span>
                      {log.data && (
                        <details className="ml-4 mt-1">
                          <summary className="text-gray-500 cursor-pointer hover:text-gray-400">
                            Show data
                          </summary>
                          <pre className="mt-1 text-gray-400 overflow-x-auto">
                            {JSON.stringify(log.data, null, 2)}
                          </pre>
                        </details>
                      )}
                    </div>
                  ))
                )}
              </div>
            </div>
          </div>

          {/* Network Info */}
          <div className="bg-gray-800 rounded-lg p-4">
            <h2 className="text-xl font-semibold mb-4">Network Information</h2>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4 text-sm">
              <div>
                <strong className="text-gray-400">Gateway URL:</strong>
                <div className="mt-1 font-mono text-xs bg-gray-900 p-2 rounded">
                  {typeof window !== 'undefined' ? window.location.origin : 'N/A'}
                </div>
              </div>
              <div>
                <strong className="text-gray-400">Status Endpoint:</strong>
                <div className="mt-1 font-mono text-xs bg-gray-900 p-2 rounded">
                  /api/ros/status
                </div>
              </div>
              <div>
                <strong className="text-gray-400">WebSocket Endpoint:</strong>
                <div className="mt-1 font-mono text-xs bg-gray-900 p-2 rounded">
                  /ws/ros/telemetry
                </div>
              </div>
              <div>
                <strong className="text-gray-400">Debug Mode:</strong>
                <div className="mt-1">
                  {DEBUG ? (
                    <span className="text-green-400">Enabled (check console for detailed logs)</span>
                  ) : (
                    <span className="text-gray-500">Disabled (set NEXT_PUBLIC_ROS_DEBUG=1 or localStorage.ros_debug=true)</span>
                  )}
                </div>
              </div>
            </div>
          </div>

          {/* DevTools Instructions */}
          <div className="mt-6 bg-blue-900/20 border border-blue-500 rounded-lg p-4">
            <h3 className="text-lg font-semibold mb-2">🔧 Using Browser DevTools</h3>
            <div className="text-sm space-y-2">
              <p><strong>Press F12</strong> to open DevTools, then:</p>
              <ul className="list-disc list-inside space-y-1 ml-4">
                <li><strong>Console Tab:</strong> View all debug logs with color-coded messages</li>
                <li><strong>Network Tab:</strong> Monitor API requests and responses</li>
                <li><strong>WS Filter:</strong> Filter by &quot;WS&quot; to see WebSocket frames</li>
                <li><strong>Performance Tab:</strong> Analyze refresh intervals and timing</li>
                <li><strong>Application Tab:</strong> Check localStorage for debug settings</li>
              </ul>
              <p className="mt-2 text-gray-300">
                All API calls and WebSocket messages are logged to the console when debug mode is enabled.
              </p>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}

