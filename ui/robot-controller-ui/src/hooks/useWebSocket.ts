/*
# File: /src/hooks/useWebSocket.ts
# Summary:
A centralized and reusable WebSocket hook for managing WebSocket connections, sending commands, 
and handling incoming messages efficiently.
*/

import { useEffect, useRef, useState } from 'react';

/**
 * Custom hook for managing WebSocket connections.
 * 
 * @param wsUrl - The WebSocket server URL.
 * @param onMessage - Callback to process incoming messages.
 * @param onOpen - Optional callback executed when the WebSocket connection is successfully established.
 * @param onClose - Optional callback executed when the WebSocket connection is closed.
 * @returns An object containing the `sendCommand` function and the WebSocket connection status.
 */
export const useWebSocket = (
  wsUrl: string,
  onMessage: (data: any) => void,
  onOpen?: () => void,
  onClose?: () => void
) => {
  const ws = useRef<WebSocket | null>(null); // Holds the WebSocket instance
  const [isConnected, setIsConnected] = useState(false); // Tracks the connection status

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket(wsUrl);

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
      setIsConnected(true);
      onOpen?.(); // Call optional onOpen callback
    };

    ws.current.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data); // Parse the incoming WebSocket message
        onMessage(data); // Trigger the onMessage callback with parsed data
      } catch (error) {
        console.error('Error parsing WebSocket message:', error);
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
      setIsConnected(false);
      onClose?.(); // Call optional onClose callback
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Clean up WebSocket connection on component unmount
    return () => {
      if (ws.current) {
        ws.current.close();
        console.log('WebSocket connection closed on cleanup');
      }
    };
  }, [wsUrl, onMessage, onOpen, onClose]);

  /**
   * Sends a command through the WebSocket connection.
   * 
   * @param command - The command to send (e.g., "move-up").
   * @param data - Optional additional payload to send with the command.
   */
  const sendCommand = (command: string, data?: Record<string, any>) => {
    if (ws.current?.readyState === WebSocket.OPEN) {
      const payload = { command, ...data };
      ws.current.send(JSON.stringify(payload)); // Send command with optional data
      console.log('Command sent:', payload);
    } else {
      console.error('WebSocket is not open. Command not sent:', command);
    }
  };

  return { sendCommand, isConnected }; // Expose `sendCommand` and connection status
};
