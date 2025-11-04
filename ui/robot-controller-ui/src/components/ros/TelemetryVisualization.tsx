'use client';

/**
 * ROS 2 Telemetry Visualization Component
 * 
 * Displays real-time telemetry from ROS 2 /omega/telemetry topic
 */

import React, { useState, useEffect, useCallback } from 'react';
import { buildGatewayUrl } from '@/config/gateway';

interface TelemetryMessage {
  topic: string;
  data: string;
  timestamp?: number;
}

interface TelemetryVisualizationProps {
  className?: string;
}

export const TelemetryVisualization: React.FC<TelemetryVisualizationProps> = ({ className = '' }) => {
  const [messages, setMessages] = useState<TelemetryMessage[]>([]);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [messageCount, setMessageCount] = useState(0);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimeout: NodeJS.Timeout | null = null;
    let reconnectAttempts = 0;
    const maxReconnectAttempts = 5;

    const connect = async () => {
      try {
        const wsUrl = await buildGatewayUrl('/ws/ros/telemetry');
        // Convert http/https to ws/wss
        const protocol = wsUrl.startsWith('https') ? 'wss' : 'ws';
        const url = wsUrl.replace(/^https?/, protocol);
        
        ws = new WebSocket(url);

        ws.onopen = () => {
          console.log('[ROS Telemetry] Connected');
          setConnected(true);
          setError(null);
          reconnectAttempts = 0;
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            const message: TelemetryMessage = {
              topic: data.topic || '/omega/telemetry',
              data: data.data || event.data,
              timestamp: Date.now(),
            };
            
            setMessages(prev => {
              const updated = [message, ...prev].slice(0, 100); // Keep last 100 messages
              return updated;
            });
            setMessageCount(prev => prev + 1);
          } catch (err) {
            // Handle plain text messages
            const message: TelemetryMessage = {
              topic: '/omega/telemetry',
              data: event.data,
              timestamp: Date.now(),
            };
            setMessages(prev => {
              const updated = [message, ...prev].slice(0, 100);
              return updated;
            });
            setMessageCount(prev => prev + 1);
          }
        };

        ws.onerror = (err) => {
          console.error('[ROS Telemetry] WebSocket error:', err);
          setError('Connection error');
          setConnected(false);
        };

        ws.onclose = () => {
          console.log('[ROS Telemetry] Disconnected');
          setConnected(false);
          
          // Attempt to reconnect
          if (reconnectAttempts < maxReconnectAttempts) {
            reconnectAttempts++;
            const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 10000);
            reconnectTimeout = setTimeout(() => {
              console.log(`[ROS Telemetry] Reconnecting (attempt ${reconnectAttempts})...`);
              connect();
            }, delay);
          } else {
            setError('Failed to connect after multiple attempts');
          }
        };
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to connect');
        setConnected(false);
      }
    };

    connect();

    return () => {
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      if (ws) ws.close();
    };
  }, []);

  const clearMessages = () => {
    setMessages([]);
    setMessageCount(0);
  };

  const formatTimestamp = (ts?: number) => {
    if (!ts) return '';
    return new Date(ts).toLocaleTimeString();
  };

  return (
    <div className={`p-4 bg-gray-900 rounded-lg text-white ${className}`}>
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <h3 className="text-lg font-semibold">ROS 2 Telemetry</h3>
          <div className={`w-2 h-2 rounded-full ${connected ? 'bg-green-500' : 'bg-red-500'}`} />
          <span className="text-xs text-gray-400">
            {connected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <span className="text-xs text-gray-400">Messages: {messageCount}</span>
          <button
            onClick={clearMessages}
            className="px-2 py-1 bg-gray-700 hover:bg-gray-600 rounded text-xs"
          >
            Clear
          </button>
        </div>
      </div>

      {error && (
        <div className="mb-4 p-2 bg-red-900 text-red-100 rounded text-sm">
          {error}
        </div>
      )}

      {!connected && !error && (
        <div className="mb-4 p-2 bg-yellow-900 text-yellow-100 rounded text-sm">
          Connecting to ROS telemetry...
        </div>
      )}

      <div className="bg-black p-3 rounded font-mono text-sm max-h-96 overflow-y-auto">
        {messages.length === 0 ? (
          <div className="text-gray-500">No messages received yet...</div>
        ) : (
          messages.map((msg, idx) => (
            <div key={idx} className="mb-2 border-b border-gray-800 pb-2">
              <div className="flex items-center justify-between mb-1">
                <span className="text-green-400">{msg.topic}</span>
                <span className="text-xs text-gray-500">{formatTimestamp(msg.timestamp)}</span>
              </div>
              <div className="text-gray-300 break-all">{msg.data}</div>
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default TelemetryVisualization;

