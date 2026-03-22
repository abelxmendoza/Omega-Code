/*
 * useGamepad.ts
 * =============
 * Reads an Xbox controller via the Web Gamepad API and sends proportional
 * movement commands over the robot's /ws/movement WebSocket.
 *
 * Standard Gamepad API mapping (Xbox One / Xbox Series):
 *   buttons[0]  A        buttons[1]  B
 *   buttons[2]  X        buttons[3]  Y
 *   buttons[4]  LB       buttons[5]  RB
 *   buttons[6]  LT (0–1) buttons[7]  RT (0–1)
 *   buttons[8]  Back     buttons[9]  Start
 *   buttons[10] L3       buttons[11] R3
 *   buttons[12] D-Up     buttons[13] D-Down
 *   buttons[14] D-Left   buttons[15] D-Right
 *   axes[0] Left-X  axes[1] Left-Y
 *   axes[2] Right-X axes[3] Right-Y
 *
 * Drive mapping (matches xbox_teleop_node.py on the Pi):
 *   RT (buttons[7].value)   → forward  (linear.x > 0)
 *   LT (buttons[6].value)   → reverse  (linear.x < 0)
 *   Left Stick X (axes[0])  → steer    (angular.z)
 *   A  (buttons[0])         → E-Stop while held
 *   Right Stick X (axes[2]) → camera pan  (reported in state for debug panel)
 *   Right Stick Y (axes[3]) → camera tilt (reported in state for debug panel)
 *   D-Pad                   → reported in state for debug panel
 */

import React, { useEffect, useRef, useState, useCallback } from 'react';

const DEAD_ZONE       = 0.12;   // axis dead zone (matches Pi node default)
const SEND_RATE_MS    = 50;     // ~20 Hz send rate
const CHANGE_THRESH   = 0.02;   // minimum change before re-sending twist
const ZERO_SEND_COUNT = 4;      // send N zero frames after last non-zero to ensure stop

function applyDead(v: number, dz = DEAD_ZONE): number {
  if (Math.abs(v) < dz) return 0;
  const sign = v > 0 ? 1 : -1;
  return sign * (Math.abs(v) - dz) / (1 - dz);
}

function btn(gp: Gamepad, idx: number): boolean {
  return (gp.buttons[idx]?.pressed) ?? false;
}

function btnVal(gp: Gamepad, idx: number): number {
  return (gp.buttons[idx]?.value) ?? 0;
}

export interface GamepadState {
  connected: boolean;
  name: string;
  index: number;
  // Drive axes (dead-zone applied, normalised -1..1)
  leftX: number;
  leftY: number;
  rightX: number;
  rightY: number;
  // Triggers (0..1)
  lt: number;
  rt: number;
  // Computed drive vector (what gets sent to the robot)
  linearX: number;
  angularZ: number;
  // Buttons
  a: boolean;
  b: boolean;
  x: boolean;
  y: boolean;
  lb: boolean;
  rb: boolean;
  dUp: boolean;
  dDown: boolean;
  dLeft: boolean;
  dRight: boolean;
  start: boolean;
  back: boolean;
  estop: boolean;
}

const DISCONNECTED_STATE: GamepadState = {
  connected: false,
  name: '',
  index: -1,
  leftX: 0, leftY: 0, rightX: 0, rightY: 0,
  lt: 0, rt: 0,
  linearX: 0, angularZ: 0,
  a: false, b: false, x: false, y: false,
  lb: false, rb: false,
  dUp: false, dDown: false, dLeft: false, dRight: false,
  start: false, back: false,
  estop: false,
};

interface UseGamepadOptions {
  /**
   * A ref to the WebSocket to send on. Using a ref (rather than the socket
   * directly) ensures the hook always sees the latest live socket across
   * reconnects without needing to recreate the hook.
   */
  wsRef: React.MutableRefObject<WebSocket | null>;
  /** When true, gamepad reads but does not send commands. */
  paused?: boolean;
}

export function useGamepad({ wsRef, paused = false }: UseGamepadOptions): GamepadState {
  const [state, setState] = useState<GamepadState>(DISCONNECTED_STATE);

  const lastSentRef    = useRef<{ lx: number; az: number }>({ lx: 0, az: 0 });
  const lastSendTimeRef = useRef(0);
  const zeroCountRef   = useRef(0);
  const rafRef         = useRef<number>(0);
  const activeIndexRef = useRef<number>(-1);

  // ---- Send twist over WS ------------------------------------------------
  // Reads wsRef.current each call so reconnects are handled transparently.
  const sendTwist = useCallback((linear_x: number, angular_z: number) => {
    if (paused) return;
    const socket = wsRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    try {
      socket.send(JSON.stringify({ command: 'twist', linear_x, angular_z }));
    } catch {
      // socket closed between check and send
    }
  }, [wsRef, paused]);

  // ---- Main poll loop (requestAnimationFrame) ----------------------------
  useEffect(() => {
    let stopped = false;

    const poll = () => {
      if (stopped) return;
      rafRef.current = requestAnimationFrame(poll);

      const gamepads = navigator.getGamepads();

      // Pick the active gamepad (prefer previously active index)
      let gp: Gamepad | null = null;
      if (activeIndexRef.current >= 0) {
        gp = gamepads[activeIndexRef.current] ?? null;
      }
      if (!gp) {
        for (const g of gamepads) {
          if (g && g.connected) { gp = g; break; }
        }
      }

      if (!gp) {
        activeIndexRef.current = -1;
        setState(DISCONNECTED_STATE);
        return;
      }

      activeIndexRef.current = gp.index;

      // --- Read axes (dead-zone applied) ---
      const leftX  = applyDead(gp.axes[0] ?? 0);
      const leftY  = applyDead(gp.axes[1] ?? 0);
      const rightX = applyDead(gp.axes[2] ?? 0);
      const rightY = applyDead(gp.axes[3] ?? 0);

      // Triggers — some browsers report them as axes[4]/axes[5],
      // others as buttons[6]/buttons[7]. Handle both.
      const lt = Math.max(btnVal(gp, 6), applyDead((gp.axes[4] ?? -1) * 0.5 + 0.5));
      const rt = Math.max(btnVal(gp, 7), applyDead((gp.axes[5] ?? -1) * 0.5 + 0.5));

      // --- Buttons ---
      const a      = btn(gp, 0);
      const b      = btn(gp, 1);
      const x      = btn(gp, 2);
      const y      = btn(gp, 3);
      const lb     = btn(gp, 4);
      const rb     = btn(gp, 5);
      const back   = btn(gp, 8);
      const start  = btn(gp, 9);
      const dUp    = btn(gp, 12);
      const dDown  = btn(gp, 13);
      const dLeft  = btn(gp, 14);
      const dRight = btn(gp, 15);
      const estop  = a;   // A button = emergency stop (matches Pi node)

      // --- Compute drive vector ---
      const rawLinear  = rt - lt;                // +1 = full forward, -1 = full reverse
      const rawAngular = -leftX;                 // left stick right → negative angular (turn right)
      const linearX  = estop ? 0 : rawLinear;
      const angularZ = estop ? 0 : rawAngular;

      // --- Throttled send ---
      const now = Date.now();
      const { lx: prevLx, az: prevAz } = lastSentRef.current;
      const changed =
        Math.abs(linearX - prevLx) > CHANGE_THRESH ||
        Math.abs(angularZ - prevAz) > CHANGE_THRESH;
      const isZero = linearX === 0 && angularZ === 0;
      const due    = (now - lastSendTimeRef.current) >= SEND_RATE_MS;

      if (due && !paused) {
        if (!isZero) {
          sendTwist(linearX, angularZ);
          lastSentRef.current   = { lx: linearX, az: angularZ };
          lastSendTimeRef.current = now;
          zeroCountRef.current  = 0;
        } else if (changed || zeroCountRef.current < ZERO_SEND_COUNT) {
          // Send a few zero frames to guarantee the robot stops
          sendTwist(0, 0);
          lastSentRef.current   = { lx: 0, az: 0 };
          lastSendTimeRef.current = now;
          zeroCountRef.current++;
        }
      }

      setState({
        connected: true,
        name: gp.id,
        index: gp.index,
        leftX, leftY, rightX, rightY,
        lt, rt,
        linearX, angularZ,
        a, b, x, y, lb, rb,
        dUp, dDown, dLeft, dRight,
        start, back,
        estop,
      });
    };

    rafRef.current = requestAnimationFrame(poll);

    return () => {
      stopped = true;
      cancelAnimationFrame(rafRef.current);
    };
  }, [sendTwist, paused]);

  // ---- Safety: send zero stop on unmount if connected -------------------
  useEffect(() => {
    return () => {
      if (state.connected) {
        sendTwist(0, 0);
      }
    };
  }, [state.connected, sendTwist]);

  return state;
}
