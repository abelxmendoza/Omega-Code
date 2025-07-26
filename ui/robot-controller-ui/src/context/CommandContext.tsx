/*
# File: /src/context/CommandContext.tsx
# Summary:
Centralized context for managing WebSocket communication and command logging with timestamps.
Features WebSocket lifecycle event handling, command history management, and offline command logging.
*/

import React, {
  createContext,
  useState,
  useContext,
  ReactNode,
  useRef,
  useEffect,
} from 'react';
import { connectMovementWs } from '@/utils/connectMovementWs';

// Define the structure of a command entry with a timestamp
interface CommandEntry {
  command: string; // The command string
  timestamp: number; // The timestamp when the command was logged
}

// Define the shape of the CommandContext
interface CommandContextType {
  sendCommand: (command: string, data?: Record<string, any>) => void; // Function to send commands
  commands: CommandEntry[]; // Array of logged commands with timestamps
  addCommand: (command: string) => void; // Function to add commands to the log
  popCommand: () => void; // Function to remove the most recent command
}

// Create the CommandContext with an undefined default value
const CommandContext = createContext<CommandContextType | undefined>(undefined);

/*
# Hook: useCommand
Provides easy access to CommandContext and ensures proper usage within its provider.
*/
export const useCommand = (): CommandContextType => {
  const context = useContext(CommandContext);
  if (!context) {
    throw new Error('useCommand must be used within CommandProvider');
  }
  return context;
};

/*
# Component: CommandProvider
Manages WebSocket connection, command logging, and state handling. Provides CommandContext to children.
*/
export const CommandProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [commands, setCommands] = useState<CommandEntry[]>([]); // Command log state
  const ws = useRef<WebSocket | null>(null); // Reference to WebSocket instance

  /**
   * Logs a command with the current timestamp.
   * @param command - The command string to log.
   */
  const addCommand = (command: string) => {
    const timestamp = Date.now();
    setCommands((prev) => [{ command, timestamp }, ...prev]); // Add to the top of the stack
    console.log(`[Command Log] ${command}`);
  };

  /**
   * Removes the most recent command from the log.
   */
  const popCommand = () => {
    if (commands.length > 0) {
      setCommands((prev) => prev.slice(1)); // Remove the latest command
    } else {
      console.warn('No commands to pop. Command log is empty.');
    }
  };

  /**
   * Sends a command via WebSocket, or logs it as offline if the WebSocket is closed.
   * @param command - The command string to send.
   * @param data - Optional additional data to send with the command.
   */
  const sendCommand = (command: string, data?: Record<string, any>) => {
    const payload = JSON.stringify({ command, ...data });
    if (ws.current?.readyState === WebSocket.OPEN) {
      ws.current.send(payload);
      addCommand(`Sent: ${command}`);
    } else {
      console.warn(`[WebSocket] Not open. Command logged offline: ${command}`);
      addCommand(`Offline Sent: ${command}`);
    }
  };

  /**
   * Establishes and manages the WebSocket connection lifecycle.
   */
  useEffect(() => {
    let shouldReconnect = true;

    const setupWebSocket = (wsInstance: WebSocket) => {
      ws.current = wsInstance;

      wsInstance.onopen = () => {
        console.log('[WebSocket] Connected');
        addCommand('WebSocket connected');
      };

      wsInstance.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.command) {
            addCommand(`Received: ${data.command}`);
          } else {
            console.warn('[WebSocket] Unrecognized message:', event.data);
          }
        } catch (error) {
          console.error('[WebSocket] Error parsing message:', error);
        }
      };

      wsInstance.onclose = () => {
        console.warn('[WebSocket] Disconnected. Attempting to reconnect...');
        addCommand('WebSocket disconnected');
        if (shouldReconnect) {
          setTimeout(() => {
            connectAndSetup();
          }, 2000);
        }
      };

      wsInstance.onerror = (event: Event) => {
        console.error('[WebSocket] Error:', event);
        const errMsg =
          event instanceof ErrorEvent ? event.message : 'Unknown WebSocket error';
        addCommand(`WebSocket error: ${errMsg}`);
      };
    };

    const connectAndSetup = () => {
      connectMovementWs()
        .then(setupWebSocket)
        .catch(() => {
          addCommand('WebSocket failed to connect (Tailscale and LAN)');
          setTimeout(connectAndSetup, 2000);
        });
    };

    connectAndSetup();

    // Cleanup WebSocket connection on component unmount
    return () => {
      shouldReconnect = false;
      if (ws.current) {
        ws.current.close();
        addCommand('WebSocket connection closed during cleanup');
      }
    };
  }, []);

  // Provide the CommandContext values to children
  return (
    <CommandContext.Provider value={{ sendCommand, commands, addCommand, popCommand }}>
      {children}
    </CommandContext.Provider>
  );
};

export { CommandContext };

