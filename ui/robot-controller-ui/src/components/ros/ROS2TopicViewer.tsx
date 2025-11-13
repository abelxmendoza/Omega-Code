/**
 * ROS2TopicViewer Component
 * 
 * Real-time viewer for ROS2 topics. Subscribe to any ROS2 topic
 * and display data in real-time.
 * 
 * Usage:
 *   <ROS2TopicViewer 
 *     topic="/omega/sensors/ultrasonic"
 *     msgType="Float32"
 *     label="Ultrasonic Distance"
 *     unit="cm"
 *   />
 */

import React, { useEffect, useState, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { buildGatewayUrl } from '@/utils/gateway';

interface ROS2TopicViewerProps {
  topic: string;
  msgType: 'String' | 'Float32' | 'Twist' | 'BatteryState' | 'Int32MultiArray';
  label?: string;
  unit?: string;
  className?: string;
}

interface ROS2Message {
  type: string;
  topic: string;
  data: any;
  timestamp: string;
}

export function ROS2TopicViewer({
  topic,
  msgType,
  label,
  unit,
  className = ''
}: ROS2TopicViewerProps) {
  const [connected, setConnected] = useState(false);
  const [latestData, setLatestData] = useState<any>(null);
  const [messageCount, setMessageCount] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectAttempts = useRef(0);

  useEffect(() => {
    let mounted = true;

    const connect = async () => {
      try {
        const wsUrl = await buildGatewayUrl('/ws/ros/bridge');
        const protocol = wsUrl.startsWith('https') ? 'wss' : 'ws';
        const url = wsUrl.replace(/^https?/, protocol);

        const ws = new WebSocket(url);
        wsRef.current = ws;

        ws.onopen = () => {
          if (!mounted) return;
          setConnected(true);
          setError(null);
          reconnectAttempts.current = 0;

          // Subscribe to topic
          ws.send(JSON.stringify({
            type: 'subscribe',
            topic: topic,
            msg_type: msgType
          }));
        };

        ws.onmessage = (event) => {
          if (!mounted) return;
          try {
            const msg: ROS2Message = JSON.parse(event.data);

            if (msg.type === 'ros2_message' && msg.topic === topic) {
              setLatestData(msg.data);
              setMessageCount(prev => prev + 1);
            } else if (msg.type === 'subscription_confirmed') {
              console.log(`Subscribed to ${topic}`);
            } else if (msg.type === 'error') {
              setError(msg.data?.error || 'Unknown error');
            }
          } catch (e) {
            console.error('Error parsing message:', e);
          }
        };

        ws.onerror = (error) => {
          if (!mounted) return;
          console.error('WebSocket error:', error);
          setError('Connection error');
          setConnected(false);
        };

        ws.onclose = () => {
          if (!mounted) return;
          setConnected(false);
          wsRef.current = null;

          // Reconnect with exponential backoff
          if (reconnectAttempts.current < 10) {
            const delay = Math.min(1000 * Math.pow(2, reconnectAttempts.current), 30000);
            reconnectAttempts.current++;
            reconnectTimeoutRef.current = setTimeout(connect, delay);
          }
        };
      } catch (err) {
        if (!mounted) return;
        setError(err instanceof Error ? err.message : 'Connection failed');
        setConnected(false);
      }
    };

    connect();

    return () => {
      mounted = false;
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, [topic, msgType]);

  const formatValue = (data: any): string => {
    if (data === null || data === undefined) return 'No data';

    if (msgType === 'Float32' && typeof data.data === 'number') {
      return `${data.data.toFixed(2)}${unit ? ` ${unit}` : ''}`;
    }

    if (msgType === 'String' && typeof data.data === 'string') {
      return data.data;
    }

    if (msgType === 'Twist') {
      return `Linear: ${data.linear?.x?.toFixed(2) || 0}, Angular: ${data.angular?.z?.toFixed(2) || 0}`;
    }

    if (msgType === 'BatteryState') {
      return `${data.voltage?.toFixed(2) || 0}V (${data.percentage?.toFixed(1) || 0}%)`;
    }

    if (msgType === 'Int32MultiArray') {
      return `[${data.data?.join(', ') || ''}]`;
    }

    return JSON.stringify(data);
  };

  return (
    <Card className={className}>
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-sm font-medium">
            {label || topic}
          </CardTitle>
          <Badge variant={connected ? 'default' : 'secondary'}>
            {connected ? 'Connected' : 'Disconnected'}
          </Badge>
        </div>
      </CardHeader>
      <CardContent>
        {error ? (
          <div className="text-sm text-red-500">{error}</div>
        ) : (
          <>
            <div className="text-2xl font-bold mb-2">
              {formatValue(latestData)}
            </div>
            <div className="text-xs text-muted-foreground">
              Messages: {messageCount} | Type: {msgType}
            </div>
          </>
        )}
      </CardContent>
    </Card>
  );
}

