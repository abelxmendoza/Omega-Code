/*
# File: /src/context/CommandContext.tsx
# Summary:
Centralized context for managing WebSocket communication and command logging with timestamps.
Features WebSocket lifecycle event handling, command history management, offline command logging,
and now exposes live connection status (connecting/connected/disconnected) plus optional latency
from a JSON ping/pong heartbeat.
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

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

// Define the structure of a command entry with a timestamp
interface CommandEntry {
  command: string;
  timestamp: number;
}

// Define the shape of the CommandContext
interface CommandContextType {
  sendCommand: (command: string, data?: Record<string, any>) => void;
  commands: CommandEntry[];
  addCommand: (command: string) => void;
  popCommand: () => void;

  // NEW: movement connection state
  status: ServerStatus;
  latencyMs: number | null; // null if unknown/degraded
  wsUrl?: string;
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
Manages WebSocket connection, command logging, status, and heartbeat. Provides CommandContext to children.
*/
export const CommandProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [commands, setCommands] = useState<CommandEntry[]>([]);
  const [status, setStatus] = useState<ServerStatus>('disconnected');
  const [latencyMs, setLatencyMs] = useState<number | null>(null);
  const [wsUrl, setWsUrl] = useState<string | undefined>(undefined);

  const ws = useRef<WebSocket | null>(null);

  // Heartbeat refs
  const hbTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const pongTimeout = useRef<ReturnType<typeof setTimeout> | null>(null);
  const pingSentAt = useRef<number | null>(null);

  // Reconnect control
  const shouldReconnect = useRef(true);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  /**
   * Logs a command with the current timestamp.
   */
  const addCommand = (command: string) => {
    const timestamp = Date.now();
    setCommands((prev) => [{ command, timestamp }, ...prev]);
    console.log(`[Command Log] ${command}`);
  };

  /**
   * Removes the most recent command from the log.
   */
  const popCommand = () => {
    setCommands((prev) => (prev.length > 0 ? prev.slice(1) : prev));
  };

  /**
   * Sends a command via WebSocket, or logs it as offline if the WebSocket is closed.
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

  // ---- Heartbeat helpers (JSON ping/pong) ----
  const cleanupHeartbeat = () => {
    if (hbTimer.current) {
      clearInterval(hbTimer.current);
      hbTimer.current = null;
    }
    if (pongTimeout.current) {
      clearTimeout(pongTimeout.current);
      pongTimeout.current = null;
    }
    pingSentAt.current = null;
  };

  const startHeartbeat = () => {
    cleanupHeartbeat();
    hbTimer.current = setInterval(() => {
      const sock = ws.current;
      if (!sock || sock.readyState !== WebSocket.OPEN) return;
      try {
        pingSentAt.current = performance.now();
        sock.send(JSON.stringify({ type: 'ping', ts: Date.now() }));

        if (pongTimeout.current) clearTimeout(pongTimeout.current);
        pongTimeout.current = setTimeout(() => {
          // No pong in time -> mark degraded latency but stay connected
          pingSentAt.current = null;
          setLatencyMs(null);
        }, 6000);
      } catch {
        setStatus('disconnected');
      }
    }, 10000);
  };

  // ---- Connection lifecycle ----
  useEffect(() => {
    shouldReconnect.current = true;

    const setupWebSocket = (wsInstance: WebSocket) => {
      ws.current = wsInstance;
      setWsUrl(wsInstance.url);

      wsInstance.onopen = () => {
        setStatus('connected');
        setLatencyMs(null);
        addCommand('WebSocket connected');
        startHeartbeat();
      };

      wsInstance.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          // Heartbeat pong
          if (data?.type === 'pong') {
            const end = performance.now();
            const start = pingSentAt.current ?? end;
            if (pongTimeout.current) {
              clearTimeout(pongTimeout.current);
              pongTimeout.current = null;
            }
            setLatencyMs(Math.max(0, Math.round(end - start)));
            pingSentAt.current = null;
            return;
          }

          // Normal echo/command log
          if (data.command) {
            addCommand(`Received: ${data.command}`);
          } else if (data?.status === 'connected' && data?.service === 'movement') {
            setStatus('connected');
          } else {
            // Unknown payloads are OK; keep quiet
          }
        } catch (error) {
          console.error('[WebSocket] Error parsing message:', error);
        }
      };

      wsInstance.onclose = () => {
        setStatus('disconnected');
        addCommand('WebSocket disconnected');
        cleanupHeartbeat();

        if (shouldReconnect.current) {
          const backoff = 2000; // keep your original cadence
          reconnectTimer.current = setTimeout(connectAndSetup, backoff);
        }
      };

      wsInstance.onerror = (event: Event) => {
        console.error('[WebSocket] Error:', event);
        const errMsg =
          event instanceof ErrorEvent ? event.message : 'Unknown WebSocket error';
        addCommand(`WebSocket error: ${errMsg}`);
        // onclose will handle reconnect
      };
    };

    const connectAndSetup = () => {
      setStatus('connecting');
      connectMovementWs()
        .then(setupWebSocket)
        .catch(() => {
          addCommand('WebSocket failed to connect (Tailscale and LAN)');
          reconnectTimer.current = setTimeout(connectAndSetup, 2000);
        });
    };

    connectAndSetup();

    // Cleanup on unmount
    return () => {
      shouldReconnect.current = false;
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
      cleanupHeartbeat();
      if (ws.current) {
        try { ws.current.close(); } catch {}
        addCommand('WebSocket connection closed during cleanup');
      }
    };
  }, []);

  return (
    <CommandContext.Provider
      value={{ sendCommand, commands, addCommand, popCommand, status, latencyMs, wsUrl }}
    >
      {children}
    </CommandContext.Provider>
  );
};

export { CommandContext };
