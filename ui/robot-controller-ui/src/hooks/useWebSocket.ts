import { useState, useEffect, useRef, useCallback } from 'react';

interface WebSocketData {
  sensors?: Record<string, any>;
  status?: {
    connected: boolean;
    battery: number;
  };
  movement?: {
    speed: number;
    direction: string;
  };
  camera?: {
    tilt: number;
    pan: number;
  };
  led?: {
    color: string;
    brightness: number;
    pattern: string;
  };
}

interface UseWebSocketOptions {
  url?: string;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
}

export const useWebSocket = (options: UseWebSocketOptions = {}) => {
  const {
    url = process.env.NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT || 'ws://localhost:8080/ws',
    reconnectInterval = 3000,
    maxReconnectAttempts = 5
  } = options;

  const [data, setData] = useState<WebSocketData>({});
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [reconnectAttempts, setReconnectAttempts] = useState(0);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const connect = useCallback(() => {
    try {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('WebSocket connected');
        setIsConnected(true);
        setError(null);
        setReconnectAttempts(0);
      };

      ws.onmessage = (event) => {
        try {
          const newData = JSON.parse(event.data);
          setData(prevData => ({ ...prevData, ...newData }));
        } catch (err) {
          console.error('Failed to parse WebSocket message:', err);
        }
      };

      ws.onclose = () => {
        console.log('WebSocket disconnected');
        setIsConnected(false);
        
        if (reconnectAttempts < maxReconnectAttempts) {
          console.log(`Attempting to reconnect... (${reconnectAttempts + 1}/${maxReconnectAttempts})`);
          reconnectTimeoutRef.current = setTimeout(() => {
            setReconnectAttempts(prev => prev + 1);
            connect();
          }, reconnectInterval);
        } else {
          setError('Max reconnection attempts reached');
        }
      };

      ws.onerror = (err) => {
        console.error('WebSocket error:', err);
        setError('WebSocket connection error');
      };

    } catch (err) {
      console.error('Failed to create WebSocket:', err);
      setError('Failed to create WebSocket connection');
    }
  }, [url, reconnectInterval, maxReconnectAttempts, reconnectAttempts]);

  const disconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
    }
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setIsConnected(false);
  }, []);

  const sendMessage = useCallback((message: any) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    } else {
      console.warn('WebSocket is not connected');
    }
  }, []);

  useEffect(() => {
    connect();
    return () => disconnect();
  }, [connect, disconnect]);

  return {
    data,
    isConnected,
    error,
    reconnectAttempts,
    sendMessage,
    connect,
    disconnect
  };
};