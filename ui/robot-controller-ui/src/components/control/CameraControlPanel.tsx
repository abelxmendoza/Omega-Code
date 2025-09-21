/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/control/CameraControlPanel.tsx
# Summary:
Pan/Tilt (camera) control with buttons + Arrow keys.
- Auto-repeat while key or pointer is held (smooth nudging)
- Shift+Arrow = larger step via {command:"servo-vertical|horizontal", angle:±N}
- Center button (C or Home)
- Safety: stops auto-repeat on blur / visibility change / pointer-cancel
*/

import React from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';

type Dir = 'up' | 'down' | 'left' | 'right';

const nudgeCmd: Record<Dir, string> = {
  up:    COMMAND.CAMERA_SERVO_UP    || 'camera-servo-up',
  down:  COMMAND.CAMERA_SERVO_DOWN  || 'camera-servo-down',
  left:  COMMAND.CAMERA_SERVO_LEFT  || 'camera-servo-left',
  right: COMMAND.CAMERA_SERVO_RIGHT || 'camera-servo-right',
};

const SERVO_H = (COMMAND as any).CMD_SERVO_HORIZONTAL || (COMMAND as any).SERVO_HORIZONTAL || 'servo-horizontal';
const SERVO_V = (COMMAND as any).CMD_SERVO_VERTICAL   || (COMMAND as any).SERVO_VERTICAL   || 'servo-vertical';
const RESET   = COMMAND.RESET_SERVO || 'reset-servo';
const FAST_STEP = 10;

const keyToDir: Record<string, Dir | undefined> = {
  ArrowUp: 'up',
  ArrowDown: 'down',
  ArrowLeft: 'left',
  ArrowRight: 'right',
};

export default function CameraControlPanel() {
  const { sendCommand, addCommand } = useCommand();

  const [pressed, setPressed] = React.useState<Record<Dir, boolean>>({
    up: false, down: false, left: false, right: false,
  });

  const repeatRef = React.useRef<ReturnType<typeof setInterval> | null>(null);
  const activeDirRef = React.useRef<Dir | null>(null);
  const shiftRef = React.useRef(false);

  const sendJson = React.useCallback((cmd: string, payload?: Record<string, unknown>) => {
    try {
      sendCommand(cmd, payload);
      addCommand(`Sent: ${cmd}${payload ? ' ' + JSON.stringify(payload) : ''}`);
    } catch (e) {
      addCommand(`Send failed: ${String(e)}`);
    }
  }, [sendCommand, addCommand]);

  const nudge = React.useCallback((dir: Dir) => {
    if (shiftRef.current) {
      if (dir === 'left')  return sendJson(SERVO_H, { angle: -FAST_STEP });
      if (dir === 'right') return sendJson(SERVO_H, { angle: +FAST_STEP });
      if (dir === 'up')    return sendJson(SERVO_V, { angle: +FAST_STEP });
      if (dir === 'down')  return sendJson(SERVO_V, { angle: -FAST_STEP });
    }
    sendJson(nudgeCmd[dir]);
  }, [sendJson]);

  const clearPrevPressed = React.useCallback((prev: Dir | null) => {
    if (!prev) return;
    setPressed(p => ({ ...p, [prev]: false }));
  }, []);

  const startRepeat = React.useCallback((dir: Dir) => {
    if (activeDirRef.current === dir) return;          // already repeating this dir
    if (repeatRef.current) clearInterval(repeatRef.current);

    // unpress previously active button so only one is lit
    clearPrevPressed(activeDirRef.current);

    activeDirRef.current = dir;
    setPressed(p => ({ ...p, [dir]: true }));

    // immediate nudge + periodic nudging
    nudge(dir);
    repeatRef.current = setInterval(() => nudge(dir), 120);
  }, [nudge, clearPrevPressed]);

  const stopRepeat = React.useCallback(() => {
    if (repeatRef.current) clearInterval(repeatRef.current);
    repeatRef.current = null;
    activeDirRef.current = null;
    setPressed({ up: false, down: false, left: false, right: false });
  }, []);

  const stopRepeatSilent = React.useCallback(() => {
    if (repeatRef.current) clearInterval(repeatRef.current);
    repeatRef.current = null;
    activeDirRef.current = null;
    // Don't call setPressed in cleanup to avoid infinite loops
  }, []);

  // --- Keyboard support ---
  React.useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      const el = document.activeElement as HTMLElement | null;
      if (el && (el.isContentEditable || el.tagName === 'INPUT' || el.tagName === 'TEXTAREA')) return;

      shiftRef.current = e.shiftKey;
      const dir = keyToDir[e.key];
      if (dir) {
        e.preventDefault();
        e.stopPropagation();
        if (e.repeat) return; // ignore OS auto-repeat; we do our own
        startRepeat(dir);
        return;
      }
      if (e.key.toLowerCase() === 'c' || e.key === 'Home') {
        e.preventDefault();
        e.stopPropagation();
        sendJson(RESET);
      }
    };

    const onKeyUp = (e: KeyboardEvent) => {
      shiftRef.current = e.shiftKey;
      const dir = keyToDir[e.key];
      // only stop if the released key is the currently active direction
      if (dir && activeDirRef.current === dir) {
        e.preventDefault();
        e.stopPropagation();
        stopRepeatSilent();
      }
    };

    window.addEventListener('keydown', onKeyDown, { passive: false });
    window.addEventListener('keyup', onKeyUp,   { passive: false });
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      stopRepeatSilent();
    };
  }, [startRepeat, stopRepeat, sendJson]);

  // safety: stop on blur / visibility change
  React.useEffect(() => {
    const bail = () => stopRepeatSilent();
    const onVis = () => { if (document.visibilityState !== 'visible') bail(); };
    window.addEventListener('blur', bail);
    document.addEventListener('visibilitychange', onVis);
    return () => {
      window.removeEventListener('blur', bail);
      document.removeEventListener('visibilitychange', onVis);
    };
  }, [stopRepeatSilent]);

  // Unified pointer bindings (mouse + touch + pen)
  const bindHold = (dir: Dir) => ({
    onPointerDown: (e: React.PointerEvent) => {
      e.preventDefault();
      (e.currentTarget as HTMLElement).setPointerCapture?.(e.pointerId);
      startRepeat(dir);
    },
    onPointerUp: (e: React.PointerEvent) => { e.preventDefault(); stopRepeatSilent(); },
    onPointerCancel: (e: React.PointerEvent) => { e.preventDefault(); stopRepeatSilent(); },
    onPointerLeave: (e: React.PointerEvent) => { e.preventDefault(); stopRepeatSilent(); },
  });

  const btnClass = (dir: Dir) =>
    `p-4 m-1 rounded-lg text-white font-semibold select-none
     w-14 h-14 flex items-center justify-center outline-none
     transition-colors duration-100
     ${pressed[dir] ? 'bg-emerald-600 ring-2 ring-emerald-300' : 'bg-zinc-800 hover:bg-zinc-700'}`;

  return (
    <div className="flex flex-col items-center">
      <div className="text-lg font-bold mb-2">Camera</div>

      <button
        className={btnClass('up')}
        aria-label="Camera Up (↑ / Shift=larger)"
        aria-pressed={pressed.up}
        title="ArrowUp (Shift=larger)"
        {...bindHold('up')}
      >↑</button>

      <div className="flex space-x-2">
        <button
          className={btnClass('left')}
          aria-label="Camera Left (← / Shift=larger)"
          aria-pressed={pressed.left}
          title="ArrowLeft (Shift=larger)"
          {...bindHold('left')}
        >←</button>

        <button
          className={btnClass('right')}
          aria-label="Camera Right (→ / Shift=larger)"
          aria-pressed={pressed.right}
          title="ArrowRight (Shift=larger)"
          {...bindHold('right')}
        >→</button>
      </div>

      <button
        className={btnClass('down')}
        aria-label="Camera Down (↓ / Shift=larger)"
        aria-pressed={pressed.down}
        title="ArrowDown (Shift=larger)"
        {...bindHold('down')}
      >↓</button>

      <div className="mt-3">
        <button
          onClick={() => sendJson(RESET)}
          className="px-3 py-2 rounded-md bg-sky-600 hover:bg-sky-500 text-white font-semibold"
          title="Center (C / Home)"
          aria-label="Center camera (C / Home)"
        >
          Center
        </button>
      </div>
    </div>
  );
}
