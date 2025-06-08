/*
# File: /src/hooks/ws_client.ts
# Summary:
Reusable WebSocket client module to manage WebSocket connections, send commands, and handle incoming messages.
Centralized handling improves maintainability and ensures consistent communication with the server.
*/

import WebSocket from 'ws'; // Import WebSocket for Node.js
import { v4 as uuidv4 } from 'uuid'; // For generating unique request IDs
import { COMMAND } from '../control_definitions'; // Import command definitions

// Type definition for the callback function to process received messages
type CommandCallback = (data: string) => void;

/*
# Class: WebSocketClient
Centralized management for WebSocket connections, message handling, and command dispatching.
*/
class WebSocketClient {
  private ws: WebSocket | null = null; // Active WebSocket instance
  private serverUrl: string; // WebSocket server URL
  private onMessageCallback: CommandCallback | null = null; // Callback for incoming messages
  private keepAliveInterval: NodeJS.Timeout | null = null; // Interval for keep-alive messages

  constructor(serverUrl: string) {
    this.serverUrl = serverUrl;
  }

  /*
  # Method: connect
  Establishes a WebSocket connection and sets up event listeners.
  - onMessageCallback: Optional callback to handle incoming messages.
  */
  connect(onMessageCallback?: CommandCallback) {
    this.onMessageCallback = onMessageCallback || null;

    // Close any existing connection before creating a new one
    if (this.ws) {
      this.close();
    }

    // Initialize a new WebSocket connection
    this.ws = new WebSocket(this.serverUrl);

    // Event: Connection established
    this.ws.on('open', () => {
      console.log('WebSocket connection established');
      this.startKeepAlive();
    });

    // Event: Incoming message
    this.ws.on('message', (data) => {
      const message = data.toString();
      console.log('Received message:', message);

      // Process message with callback if provided
      if (this.onMessageCallback) {
        try {
          this.onMessageCallback(message);
        } catch (err) {
          console.error('Error processing WebSocket message:', err);
        }
      }
    });

    // Event: Connection closed
    this.ws.on('close', () => {
      console.warn('WebSocket connection closed. Attempting to reconnect...');
      this.cleanup();
      setTimeout(() => this.connect(onMessageCallback), 1000); // Auto-reconnect after delay
    });

    // Event: WebSocket error
    this.ws.on('error', (error) => {
      console.error('WebSocket encountered an error:', error);
      this.cleanup();
    });
  }

  /*
  # Method: sendCommand
  Sends a command to the WebSocket server.
  - command: The command string (e.g., 'move-up').
  - angle: Optional numerical value for commands requiring an angle.
  */
  sendCommand(command: string, angle: number = 0) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      const requestId = uuidv4(); // Generate unique request ID
      const payload = { command, angle, request_id: requestId };

      this.ws.send(JSON.stringify(payload));
      console.log('Command sent:', payload);
    } else {
      console.error('WebSocket is not open. Failed to send command:', command);
    }
  }

  /*
  # Method: close
  Gracefully closes the WebSocket connection and cleans up resources.
  */
  close() {
    this.cleanup();
    if (this.ws) {
      this.ws.close();
      this.ws = null;
      console.log('WebSocket connection closed manually.');
    }
  }

  /*
  # Method: startKeepAlive
  Sends periodic keep-alive messages to maintain the WebSocket connection.
  */
  private startKeepAlive() {
    this.keepAliveInterval = setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: 'keep-alive' }));
        console.log('Keep-alive message sent.');
      }
    }, 25000); // Keep-alive interval (25 seconds)
  }

  /*
  # Method: cleanup
  Clears keep-alive intervals and resets the internal WebSocket state.
  */
  private cleanup() {
    if (this.keepAliveInterval) {
      clearInterval(this.keepAliveInterval);
      this.keepAliveInterval = null;
    }
  }
}

// Export a singleton instance of WebSocketClient
export const wsClient = new WebSocketClient('wss://100.82.88.25:8080/ws');
