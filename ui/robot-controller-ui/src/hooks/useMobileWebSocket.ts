/**
 * Mobile-Optimized WebSocket Hook
 * 
 * Features:
 * - Automatic reconnection with exponential backoff
 * - Connection health monitoring
 * - Mobile network optimization
 * - Bandwidth-aware message handling
 * - Offline/online detection
 */

import { useEffect, useRef, useState, useCallback } from 'react';
import { connectionHealth, getConnectionOptimizations, isMobileConnection } from '@/utils/connectionHealth';

export interface WebSocketState {
  status: 'connecting' | 'connected' | 'disconnected' | 'reconnecting' | 'error';
  lastMessage: any;
  error: string | null;
  connectionQuality: 'excellent' | 'good' | 'fair' | 'poor' | 'unknown';
  retryCount: number;
  latency: number;
}

export interface MobileWebSocketOptions {
  url: string;
  protocols?: string[];
  heartbeatInterval?: number;
  maxRetries?: number;
  enableCompression?: boolean;
  onMessage?: (data: any) => void;
  onError?: (error: Event) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
}

export function useMobileWebSocket(options: MobileWebSocketOptions) {
  const {
    url,
    protocols,
    heartbeatInterval = 30000,
    maxRetries = 5,
    enableCompression = true,
    onMessage,
    onError,
    onConnect,
    onDisconnect
  } = options;

  const [state, setState] = useState<WebSocketState>({
    status: 'disconnected',
    lastMessage: null,
    error: null,
    connectionQuality: 'unknown',
    retryCount: 0,
    latency: 0
  });

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const heartbeatTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const retryCountRef = useRef(0);
  const lastPingTimeRef = useRef(0);
  const isManualCloseRef = useRef(false);

  // Get mobile-optimized settings
  const optimizations = getConnectionOptimizations();
  const isMobile = isMobileConnection();

  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      return;
    }

    setState(prev => ({ ...prev, status: 'connecting', error: null }));

    try {
      const ws = new WebSocket(url, protocols);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('[MobileWebSocket] Connected to:', url);
        setState(prev => ({ 
          ...prev, 
          status: 'connected', 
          retryCount: 0,
          error: null 
        }));
        retryCountRef.current = 0;
        isManualCloseRef.current = false;
        
        // Start heartbeat
        startHeartbeat();
        
        // Update connection quality
        updateConnectionQuality();
        
        onConnect?.();
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          
          // Handle ping/pong for latency measurement
          if (data.type === 'ping') {
            ws.send(JSON.stringify({ type: 'pong', timestamp: Date.now() }));
            return;
          }
          
          if (data.type === 'pong' && lastPingTimeRef.current > 0) {
            const latency = Date.now() - lastPingTimeRef.current;
            setState(prev => ({ ...prev, latency }));
            lastPingTimeRef.current = 0;
            return;
          }
          
          setState(prev => ({ ...prev, lastMessage: data }));
          onMessage?.(data);
        } catch (error) {
          console.warn('[MobileWebSocket] Failed to parse message:', error);
        }
      };

      ws.onclose = (event) => {
        console.log('[MobileWebSocket] Connection closed:', event.code, event.reason);
        
        if (!isManualCloseRef.current) {
          setState(prev => ({ 
            ...prev, 
            status: 'disconnected',
            error: `Connection closed: ${event.reason || 'Unknown reason'}`
          }));
          
          // Attempt reconnection
          scheduleReconnect();
        }
        
        onDisconnect?.();
      };

      ws.onerror = (event) => {
        console.error('[MobileWebSocket] WebSocket error:', event);
        setState(prev => ({ 
          ...prev, 
          status: 'error',
          error: 'WebSocket connection error'
        }));
        onError?.(event);
      };

    } catch (error) {
      console.error('[MobileWebSocket] Failed to create WebSocket:', error);
      setState(prev => ({ 
        ...prev, 
        status: 'error',
        error: `Failed to connect: ${error}`
      }));
      scheduleReconnect();
    }
  }, [url, protocols, onMessage, onError, onConnect, onDisconnect]);

  const disconnect = useCallback(() => {
    isManualCloseRef.current = true;
    
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    if (heartbeatTimeoutRef.current) {
      clearTimeout(heartbeatTimeoutRef.current);
      heartbeatTimeoutRef.current = null;
    }
    
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    
    setState(prev => ({ ...prev, status: 'disconnected' }));
  }, []);

  const sendMessage = useCallback((message: any) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      try {
        const data = typeof message === 'string' ? message : JSON.stringify(message);
        wsRef.current.send(data);
        return true;
      } catch (error) {
        console.error('[MobileWebSocket] Failed to send message:', error);
        return false;
      }
    }
    return false;
  }, []);

  const scheduleReconnect = useCallback(() => {
    if (isManualCloseRef.current || retryCountRef.current >= maxRetries) {
      return;
    }

    retryCountRef.current++;
    setState(prev => ({ 
      ...prev, 
      status: 'reconnecting',
      retryCount: retryCountRef.current
    }));

    // Exponential backoff with jitter for mobile networks
    const baseDelay = isMobile ? 2000 : 1000; // Longer delay for mobile
    const delay = Math.min(
      baseDelay * Math.pow(2, retryCountRef.current - 1),
      30000 // Max 30 seconds
    );
    const jitter = Math.random() * 1000; // Add up to 1 second of jitter

    console.log(`[MobileWebSocket] Scheduling reconnect in ${delay + jitter}ms (attempt ${retryCountRef.current}/${maxRetries})`);

    reconnectTimeoutRef.current = setTimeout(() => {
      connect();
    }, delay + jitter);
  }, [connect, maxRetries, isMobile]);

  const startHeartbeat = useCallback(() => {
    if (heartbeatTimeoutRef.current) {
      clearTimeout(heartbeatTimeoutRef.current);
    }

    const sendPing = () => {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        lastPingTimeRef.current = Date.now();
        sendMessage({ type: 'ping', timestamp: Date.now() });
      }
    };

    // Send initial ping
    sendPing();

    // Schedule next ping
    heartbeatTimeoutRef.current = setTimeout(() => {
      startHeartbeat();
    }, heartbeatInterval);
  }, [sendMessage, heartbeatInterval]);

  const updateConnectionQuality = useCallback(async () => {
    try {
      await connectionHealth.startMonitoring();
      const health = connectionHealth.getCurrentHealth();
      
      if (health) {
        setState(prev => ({ 
          ...prev, 
          connectionQuality: health.status === 'disconnected' ? 'unknown' : health.status
        }));
      }
    } catch (error) {
      console.warn('[MobileWebSocket] Failed to update connection quality:', error);
    }
  }, []);

  // Online/offline detection
  useEffect(() => {
    const handleOnline = () => {
      console.log('[MobileWebSocket] Network back online');
      if (state.status === 'disconnected' || state.status === 'error') {
        retryCountRef.current = 0; // Reset retry count
        connect();
      }
    };

    const handleOffline = () => {
      console.log('[MobileWebSocket] Network offline');
      setState(prev => ({ 
        ...prev, 
        status: 'disconnected',
        error: 'Network offline'
      }));
    };

    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, [connect, state.status]);

  // Auto-connect on mount
  useEffect(() => {
    connect();
    
    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      disconnect();
      connectionHealth.stopMonitoring();
    };
  }, [disconnect]);

  return {
    ...state,
    connect,
    disconnect,
    sendMessage,
    isConnected: state.status === 'connected',
    isConnecting: state.status === 'connecting' || state.status === 'reconnecting'
  };
}
