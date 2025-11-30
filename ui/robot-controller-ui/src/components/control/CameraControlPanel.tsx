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
  const { sendCommand, addCommand, status } = useCommand();

  const [pressed, setPressed] = React.useState<Record<Dir, boolean>>({
    up: false, down: false, left: false, right: false,
  });

  const repeatRef = React.useRef<ReturnType<typeof setInterval> | null>(null);
  const activeDirRef = React.useRef<Dir | null>(null);
  const shiftRef = React.useRef(false);
  const disabled = status !== 'connected';

  const sendJson = React.useCallback((cmd: string, payload?: Record<string, unknown>) => {
    if (disabled) {
      addCommand(`Cannot send ${cmd}: WebSocket not connected`);
      return;
    }
    try {
      sendCommand(cmd, payload);
      addCommand(`Sent: ${cmd}${payload ? ' ' + JSON.stringify(payload) : ''}`);
    } catch (e) {
      addCommand(`Send failed: ${String(e)}`);
    }
  }, [sendCommand, addCommand, disabled]);

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
    if (disabled) return; // Don't start if disconnected
    if (activeDirRef.current === dir) return;          // already repeating this dir
    if (repeatRef.current) clearInterval(repeatRef.current);

    // unpress previously active button so only one is lit
    clearPrevPressed(activeDirRef.current);

    activeDirRef.current = dir;
    setPressed(p => ({ ...p, [dir]: true }));

    // immediate nudge + periodic nudging
    nudge(dir);
    repeatRef.current = setInterval(() => nudge(dir), 120);
  }, [nudge, clearPrevPressed, disabled]);

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
    setPressed({ up: false, down: false, left: false, right: false });
  }, []);

  // --- Keyboard support ---
  React.useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (disabled) return; // Don't process keyboard input when disconnected
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
  }, [startRepeat, stopRepeat, sendJson, disabled]);

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

  // Color mapping for camera directions - always show colors
  const directionColors: Record<Dir, { normal: string; pressed: string }> = {
    up: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
    down: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
    left: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
    right: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
  };

  const btnClass = (dir: Dir) => {
    const colors = directionColors[dir];
    const baseColor = pressed[dir] ? colors.pressed : colors.normal;
    return `p-4 m-1 rounded-lg text-white font-semibold select-none
     w-14 h-14 flex items-center justify-center outline-none
     transition-colors duration-100
     ${baseColor} ${
       disabled 
         ? 'cursor-not-allowed opacity-50' 
         : pressed[dir]
         ? 'ring-2'
         : 'hover:opacity-90'
     }`;
  };

  const statusText = status === 'connected' ? 'Connected' : status === 'connecting' ? 'Connecting...' : 'Disconnected';
  const statusColor = status === 'connected' ? 'text-emerald-500' : status === 'connecting' ? 'text-slate-500' : 'text-rose-500';

  return (
    <div className={`flex flex-col items-center ${disabled ? 'opacity-50' : ''}`}>
      <div className="w-full flex items-center justify-between mb-2">
        <div className="text-lg font-bold">Camera</div>
        <div className="flex items-center gap-2">
          <span className={`inline-block rounded-full ${status === 'connected' ? 'bg-emerald-500' : status === 'connecting' ? 'bg-slate-500' : 'bg-rose-500'}`} style={{ width: 8, height: 8 }} title={`Movement server: ${status}`} />
          <span className={`text-xs font-medium ${statusColor}`}>{statusText}</span>
        </div>
      </div>

      <button
        disabled={disabled}
        className={btnClass('up')}
        aria-label="Camera Up (↑ / Shift=larger)"
        aria-pressed={pressed.up}
        title={`ArrowUp (Shift=larger)${disabled ? ' - Disconnected' : ''}`}
        {...(disabled ? {} : bindHold('up'))}
      >↑</button>

      <div className="flex space-x-2">
        <button
          disabled={disabled}
          className={btnClass('left')}
          aria-label="Camera Left (← / Shift=larger)"
          aria-pressed={pressed.left}
          title={`ArrowLeft (Shift=larger)${disabled ? ' - Disconnected' : ''}`}
          {...(disabled ? {} : bindHold('left'))}
        >←</button>

        <button
          disabled={disabled}
          className={btnClass('right')}
          aria-label="Camera Right (→ / Shift=larger)"
          aria-pressed={pressed.right}
          title={`ArrowRight (Shift=larger)${disabled ? ' - Disconnected' : ''}`}
          {...(disabled ? {} : bindHold('right'))}
        >→</button>
      </div>

      <button
        disabled={disabled}
        className={btnClass('down')}
        aria-label="Camera Down (↓ / Shift=larger)"
        aria-pressed={pressed.down}
        title={`ArrowDown (Shift=larger)${disabled ? ' - Disconnected' : ''}`}
        {...(disabled ? {} : bindHold('down'))}
      >↓</button>

      <div className="mt-3">
        <button
          disabled={disabled}
          onClick={() => {
            // Center pan to 70° (your specified center position)
            sendJson(SERVO_H, { angle: 70 });
            // Center tilt to 20° (your specified center position)
            sendJson(SERVO_V, { angle: 20 });
          }}
          className={`px-3 py-2 rounded-md text-white font-semibold transition-colors ${
            disabled ? 'bg-sky-800 cursor-not-allowed opacity-50' : 'bg-sky-600 hover:bg-sky-500'
          }`}
          title={`Center camera to Pan: 70°, Tilt: 20° (C / Home)${disabled ? ' - Disconnected' : ''}`}
          aria-label={`Center camera to Pan: 70°, Tilt: 20° (C / Home)${disabled ? ' - Disconnected' : ''}`}
        >
          Center
        </button>
      </div>
    </div>
  );
}
