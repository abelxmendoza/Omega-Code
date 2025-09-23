import { useEffect, useRef, useState } from 'react';

interface RobustWebSocketOptions {
  url: string;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
  onOpen?: () => void;
  onClose?: () => void;
  onError?: (error: Event | Error) => void;
  onMessage?: (data: any) => void;
}

interface WebSocketError {
  type: 'connection' | 'message' | 'send' | 'reconnect';
  message: string;
  timestamp: number;
  url?: string;
}

export function useRobustWebSocket(options: RobustWebSocketOptions) {
  const {
    url,
    reconnectInterval = 3000,
    maxReconnectAttempts = 5,
    onOpen,
    onClose,
    onError,
    onMessage,
  } = options;

  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected' | 'error'>('disconnected');
  const [lastError, setLastError] = useState<WebSocketError | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectAttemptsRef = useRef(0);
  const isManualDisconnectRef = useRef(false);

  const connect = () => {
    try {
      // Validate URL
      if (!url || typeof url !== 'string') {
        const error: WebSocketError = {
          type: 'connection',
          message: 'Invalid WebSocket URL provided',
          timestamp: Date.now(),
          url
        };
        setLastError(error);
        setConnectionStatus('error');
        onError?.(new Error(error.message));
        return;
      }

      setConnectionStatus('connecting');
      setLastError(null);
      
      const ws = new WebSocket(url);
      
      ws.onopen = () => {
        setIsConnected(true);
        setConnectionStatus('connected');
        reconnectAttemptsRef.current = 0;
        setLastError(null);
        onOpen?.();
      };

      ws.onclose = (event) => {
        setIsConnected(false);
        setConnectionStatus('disconnected');
        onClose?.();
        
        // Only attempt to reconnect if it wasn't a manual disconnect
        if (!isManualDisconnectRef.current && reconnectAttemptsRef.current < maxReconnectAttempts) {
          reconnectAttemptsRef.current++;
          const error: WebSocketError = {
            type: 'reconnect',
            message: `Connection lost. Attempting reconnect ${reconnectAttemptsRef.current}/${maxReconnectAttempts}`,
            timestamp: Date.now(),
            url
          };
          setLastError(error);
          
          reconnectTimeoutRef.current = setTimeout(() => {
            connect();
          }, reconnectInterval);
        } else if (reconnectAttemptsRef.current >= maxReconnectAttempts) {
          const error: WebSocketError = {
            type: 'connection',
            message: `Max reconnection attempts (${maxReconnectAttempts}) exceeded`,
            timestamp: Date.now(),
            url
          };
          setLastError(error);
          setConnectionStatus('error');
          onError?.(new Error(error.message));
        }
      };

      ws.onerror = (error) => {
        const wsError: WebSocketError = {
          type: 'connection',
          message: `WebSocket error: ${error.type || 'Unknown error'}`,
          timestamp: Date.now(),
          url
        };
        setLastError(wsError);
        setConnectionStatus('error');
        onError?.(error);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          onMessage?.(data);
        } catch (error) {
          // If not JSON, pass raw data
          const wsError: WebSocketError = {
            type: 'message',
            message: `Failed to parse message as JSON: ${error instanceof Error ? error.message : 'Unknown error'}`,
            timestamp: Date.now(),
            url
          };
          setLastError(wsError);
          onMessage?.(event.data);
        }
      };

      wsRef.current = ws;
    } catch (error) {
      const wsError: WebSocketError = {
        type: 'connection',
        message: `Failed to create WebSocket: ${error instanceof Error ? error.message : 'Unknown error'}`,
        timestamp: Date.now(),
        url
      };
      setLastError(wsError);
      setConnectionStatus('error');
      onError?.(error instanceof Error ? error : new Error(String(error)));
    }
  };

  const disconnect = () => {
    isManualDisconnectRef.current = true;
    
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    if (wsRef.current) {
      try {
        wsRef.current.close();
      } catch (error) {
        console.warn('Error closing WebSocket:', error);
      }
      wsRef.current = null;
    }
    
    setIsConnected(false);
    setConnectionStatus('disconnected');
  };

  const sendMessage = (message: any) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      const error: WebSocketError = {
        type: 'send',
        message: 'WebSocket is not connected. Cannot send message.',
        timestamp: Date.now(),
        url
      };
      setLastError(error);
      onError?.(new Error(error.message));
      return false;
    }

    try {
      const data = typeof message === 'string' ? message : JSON.stringify(message);
      wsRef.current.send(data);
      return true;
    } catch (error) {
      const wsError: WebSocketError = {
        type: 'send',
        message: `Failed to send message: ${error instanceof Error ? error.message : 'Unknown error'}`,
        timestamp: Date.now(),
        url
      };
      setLastError(wsError);
      onError?.(error instanceof Error ? error : new Error(String(error)));
      return false;
    }
  };

  useEffect(() => {
    isManualDisconnectRef.current = false;
    connect();
    return () => {
      disconnect();
    };
  }, [url]); // eslint-disable-line react-hooks/exhaustive-deps

  return {
    isConnected,
    connectionStatus,
    lastError,
    reconnectAttempts: reconnectAttemptsRef.current,
    sendMessage,
    connect,
    disconnect,
  };
}
