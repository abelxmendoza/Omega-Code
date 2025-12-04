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

type ServoTelemetrySource = 'status' | 'ack';

interface ServoTelemetry {
  horizontal: number | null;
  vertical: number | null;
  min: number | null;
  max: number | null;
  updatedAt: number | null;
  source: ServoTelemetrySource | null;
}

// Movement V2 data structure
export interface MovementV2Data {
  enabled: boolean;
  profile?: {
    name: string;
    config?: any;
  };
  thermal?: {
    enabled: boolean;
    state: 'ok' | 'warning' | 'throttle' | 'kill';
    max_temp_seen: number;
    limits: {
      max_temp: number;
      warning_temp: number;
    };
  };
  ramping?: {
    current_pwm: number;
    target_pwm: number;
    is_ramping: boolean;
  };
  watchdog?: {
    enabled: boolean;
    time_until_trigger: number;
    state: string;
  };
  pid?: {
    enabled: boolean;
    target_rpm?: number;
    tuning?: {
      kp: number;
      ki: number;
      kd: number;
      kf: number;
    };
    available?: boolean;
  };
}

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

  servoTelemetry: ServoTelemetry;
  requestStatus: (reason?: string) => void;
  
  // Speed tracking
  speed: number; // Current speed in PWM (0-4095)
  speedPct: number; // Current speed as percentage (0-100)
  
  // Movement V2 data
  movementV2: MovementV2Data | null;
}

// Create the CommandContext with an undefined default value
const CommandContext = createContext<CommandContextType | undefined>(undefined);

const H_SERVO_ACTIONS = new Set([
  'servo-horizontal',
  'camera-servo-left',
  'camera-servo-right',
  'set-servo-position',
  'reset-servo',
]);

const V_SERVO_ACTIONS = new Set([
  'servo-vertical',
  'camera-servo-up',
  'camera-servo-down',
  'set-servo-position',
  'reset-servo',
]);

const isNumber = (value: unknown): value is number =>
  typeof value === 'number' && Number.isFinite(value);

const pickNumber = (...values: unknown[]): number | undefined => {
  for (const value of values) {
    if (isNumber(value)) return value;
  }
  return undefined;
};

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
  const [servoTelemetry, setServoTelemetry] = useState<ServoTelemetry>({
    horizontal: null,
    vertical: null,
    min: null,
    max: null,
    updatedAt: null,
    source: null,
  });
  
  // Speed tracking (PWM 0-4095, percentage 0-100)
  // Default to 1200 PWM (matches backend default) = ~29%
  const [speed, setSpeed] = useState<number>(1200); // Default speed matches backend
  const speedPct = Math.round((speed / 4095) * 100);
  
  // Movement V2 data
  const [movementV2, setMovementV2] = useState<MovementV2Data | null>(null);

  const ws = useRef<WebSocket | null>(null);

  // Heartbeat refs
  const hbTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const pongTimeout = useRef<ReturnType<typeof setTimeout> | null>(null);
  const pingSentAt = useRef<number | null>(null);

  // Reconnect control
  const shouldReconnect = useRef(true);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const backoffRef = useRef(1000); // Start at 1s, exponential backoff
  
  // Connection verification: wait for server confirmation before marking as connected
  const connectionVerified = useRef(false);
  const verificationTimeout = useRef<ReturnType<typeof setTimeout> | null>(null);

  /**
   * Logs a command with the current timestamp.
   */
  const addCommand = React.useCallback((command: string) => {
    const timestamp = Date.now();
    setCommands((prev) => [{ command, timestamp }, ...prev]);
    console.log(`[Command Log] ${command}`);
  }, []);

  /**
   * Removes the most recent command from the log.
   */
  const popCommand = React.useCallback(() => {
    setCommands((prev) => (prev.length > 0 ? prev.slice(1) : prev));
  }, []);

  /**
   * Sends a command via WebSocket, or logs it as offline if the WebSocket is closed.
   */
  const sendCommand = React.useCallback((command: string, data?: Record<string, any>) => {
    const payload = JSON.stringify({ command, ...data });
    if (ws.current?.readyState === WebSocket.OPEN) {
      ws.current.send(payload);
      addCommand(`Sent: ${command}`);
    } else {
      console.warn(`[WebSocket] Not open. Command logged offline: ${command}`);
      addCommand(`Offline Sent: ${command}`);
    }
  }, [addCommand]);

  const applyServoTelemetry = React.useCallback(
    (raw: any, source: ServoTelemetrySource) => {
      if (!raw) return;

      const servo = raw?.servo && typeof raw.servo === 'object' ? raw.servo : raw;
      const action = typeof raw?.action === 'string' ? raw.action : null;

      let horizontal = pickNumber(servo?.horizontal, raw?.horizontal);
      let vertical = pickNumber(servo?.vertical, raw?.vertical);
      const min = pickNumber(
        servo?.min,
        raw?.min,
        servo?.range?.min,
        raw?.servoMin,
        raw?.range?.min,
      );
      const max = pickNumber(
        servo?.max,
        raw?.max,
        servo?.range?.max,
        raw?.servoMax,
        raw?.range?.max,
      );

      if (isNumber(raw?.angle)) {
        if (horizontal === undefined && action && H_SERVO_ACTIONS.has(action)) {
          horizontal = raw.angle;
        }
        if (vertical === undefined && action && V_SERVO_ACTIONS.has(action)) {
          vertical = raw.angle;
        }
      }

      if (
        horizontal === undefined &&
        vertical === undefined &&
        min === undefined &&
        max === undefined
      ) {
        return;
      }

      setServoTelemetry((prev) => ({
        horizontal: horizontal ?? prev.horizontal,
        vertical: vertical ?? prev.vertical,
        min: min ?? prev.min,
        max: max ?? prev.max,
        updatedAt: Date.now(),
        source,
      }));
    },
    [setServoTelemetry],
  );

  const requestStatus = React.useCallback(
    (reason?: string) => {
      const suffix = reason ? ` (${reason})` : '';
      if (ws.current?.readyState === WebSocket.OPEN) {
        try {
          ws.current.send(JSON.stringify({ command: 'status' }));
          addCommand(`Sent: status${suffix}`);
        } catch (error) {
          addCommand(`Failed to send status${suffix}: ${String(error)}`);
        }
      } else {
        addCommand(`Failed to send status${suffix}: WebSocket is not open`);
      }
    },
    [addCommand],
  );

  // ---- Heartbeat helpers (JSON ping/pong) ----
  const cleanupHeartbeat = React.useCallback(() => {
    if (hbTimer.current) {
      clearInterval(hbTimer.current);
      hbTimer.current = null;
    }
    if (pongTimeout.current) {
      clearTimeout(pongTimeout.current);
      pongTimeout.current = null;
    }
    pingSentAt.current = null;
  }, []);

  const startHeartbeat = React.useCallback(() => {
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
  }, [cleanupHeartbeat]);

  // ---- Connection lifecycle ----
  useEffect(() => {
    shouldReconnect.current = true;

    const setupWebSocket = (wsInstance: WebSocket) => {
      ws.current = wsInstance;
      setWsUrl(wsInstance.url);

      wsInstance.onopen = () => {
        // Reset backoff on successful connection
        backoffRef.current = 1000;
        
        // Don't mark as connected yet - wait for verification
        connectionVerified.current = false;
        setStatus('connecting'); // Keep as connecting until verified
        setLatencyMs(null);
        addCommand('WebSocket opened, verifying connection...');

        // Set a timeout: if we don't get any message within 3 seconds, mark as disconnected
        if (verificationTimeout.current) {
          clearTimeout(verificationTimeout.current);
        }
        verificationTimeout.current = setTimeout(() => {
          if (!connectionVerified.current && ws.current === wsInstance) {
            addCommand('Connection verification timeout - server may not be responding');
            setStatus('disconnected');
            try { wsInstance.close(); } catch {}
          }
        }, 3000);
        
        // Start heartbeat which will also help verify the connection
        startHeartbeat();
        requestStatus('auto');
      };

      wsInstance.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          // Verify connection on first message received
          if (!connectionVerified.current && ws.current === wsInstance) {
            connectionVerified.current = true;
            if (verificationTimeout.current) {
              clearTimeout(verificationTimeout.current);
              verificationTimeout.current = null;
            }
            setStatus('connected');
            addCommand('WebSocket connection verified');
            // Request status to sync speed and other state
            requestStatus('connection verified');
          }

          // Welcome message from server
          if (data?.type === 'welcome') {
            addCommand(`Connected to ${data?.service || 'server'}: ${data?.status || 'connected'}`);
            return;
          }

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

          if (data?.type === 'status') {
            // Validate status payload
            if (!data || typeof data !== 'object') {
              console.error('❌ [WS][STATUS] Invalid status payload:', data);
              return;
            }
            
            addCommand('Received: status snapshot');
            applyServoTelemetry(data, 'status');
            // Update speed from status
            if (typeof data?.speed === 'number') {
              setSpeed(data.speed);
            }
            // Update Movement V2 data from status
            if (data.movementV2) {
              setMovementV2(data.movementV2);
            } else {
              console.warn('⚠️ [WS][STATUS] MovementV2 not provided by backend.');
            }
            // Also include PID data if available (PID is separate from movementV2 in backend)
            if (data.pid && data.movementV2) {
              setMovementV2({
                ...data.movementV2,
                pid: data.pid
              });
            }
            return;
          }

          if (data?.type === 'ack') {
            const action = typeof data?.action === 'string' ? data.action : null;
            if (action) {
              if (data?.status === 'error') {
                const detail = data?.error ? ` (${data.error})` : '';
                addCommand(`Ack error: ${action}${detail}`);
              } else {
                addCommand(`Ack: ${action}`);
              }
            }
            if (data?.status === 'ok') {
              applyServoTelemetry(data, 'ack');
              // Update speed from ack responses (set-speed, increase-speed, decrease-speed, stop)
              if (typeof data?.speed === 'number') {
                setSpeed(data.speed);
              }
            }
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

      wsInstance.onclose = (e: CloseEvent) => {
        console.warn(`⚠️ [WS][CLOSE] Connection closed: ${e.code} ${e.reason || ''}`);
        connectionVerified.current = false;
        if (verificationTimeout.current) {
          clearTimeout(verificationTimeout.current);
          verificationTimeout.current = null;
        }
        setStatus('disconnected');
        addCommand('WebSocket disconnected');
        cleanupHeartbeat();

        if (shouldReconnect.current) {
          // Exponential backoff: 1s, 2s, 4s, 8s, max 10s
          const delay = Math.min(backoffRef.current, 10000);
          reconnectTimer.current = setTimeout(connectAndSetup, delay);
          backoffRef.current = Math.min(backoffRef.current * 2, 10000);
        }
      };

      wsInstance.onerror = (event: Event) => {
        console.error('🔥 [WS][ERROR] WebSocket error:', event);
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
        .then((wsInstance) => {
          // Reset backoff on successful connection
          backoffRef.current = 1000;
          setupWebSocket(wsInstance);
        })
        .catch((err) => {
          const errorMsg = err?.message || String(err) || 'Unknown error';
          console.error('[CommandContext] WebSocket connection failed:', errorMsg);
          addCommand(`WebSocket failed to connect: ${errorMsg}`);
          
          // Exponential backoff: 1s, 2s, 4s, 8s, max 10s
          const delay = Math.min(backoffRef.current, 10000);
          reconnectTimer.current = setTimeout(connectAndSetup, delay);
          backoffRef.current = Math.min(backoffRef.current * 2, 10000);
        });
    };

    connectAndSetup();

    // Cleanup on unmount
    return () => {
      shouldReconnect.current = false;
      connectionVerified.current = false;
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
      if (verificationTimeout.current) {
        clearTimeout(verificationTimeout.current);
        verificationTimeout.current = null;
      }
      cleanupHeartbeat();
      if (ws.current) {
        try { ws.current.close(); } catch {}
        addCommand('WebSocket connection closed during cleanup');
      }
    };
  }, [addCommand, applyServoTelemetry, cleanupHeartbeat, requestStatus, startHeartbeat]);

  return (
    <CommandContext.Provider
      value={{
        sendCommand,
        commands,
        addCommand,
        popCommand,
        status,
        latencyMs,
        wsUrl,
        servoTelemetry,
        requestStatus,
        speed,
        speedPct,
        movementV2,
      }}
    >
      {children}
    </CommandContext.Provider>
  );
};

export { CommandContext };

