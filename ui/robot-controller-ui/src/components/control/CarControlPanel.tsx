/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/control/CarControlPanel.tsx
# Summary:
Movement control with keyboard + pointer (no Arrow keys).
- Keys: W/A/S/D for directions, Space = emergency stop
- Only one button shows as "pressed" at a time (matches Camera panel behavior)
- Ignores OS key-repeat (we manage our own pressed state)
- Auto-stop on keyup of the *active* key, blur, or visibility change
*/

import React, { useState, useCallback, useEffect, useRef } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';

const directions = ['up', 'down', 'left', 'right'] as const;
type Direction = typeof directions[number];

const keyToDirection: Record<string, Direction | undefined> = {
  w: 'up',
  a: 'left',
  s: 'down',
  d: 'right',
};

const directionToCommand: Record<Direction, string> = {
  up:    COMMAND.MOVE_UP    || 'move-up',
  down:  COMMAND.MOVE_DOWN  || 'move-down',
  left:  COMMAND.MOVE_LEFT  || 'move-left',
  right: COMMAND.MOVE_RIGHT || 'move-right',
};

const MOVE_STOP = COMMAND.MOVE_STOP || 'move-stop';

const CarControlPanel: React.FC = () => {
  const { sendCommand, addCommand, status } = useCommand();

  const [pressed, setPressed] = useState<Record<Direction, boolean>>({
    up: false, down: false, left: false, right: false,
  });

  const activeDirRef = useRef<Direction | null>(null);
  const disabled = status !== 'connected';

  const send = useCallback((cmd: string) => {
    if (disabled) {
      addCommand(`Cannot send ${cmd}: WebSocket not connected`);
      return;
    }
    try {
      sendCommand(cmd);
      addCommand(`Command Sent: ${cmd}`);
    } catch (e: any) {
      addCommand(`Command Error: ${cmd} — ${e?.message ?? e}`);
    }
  }, [sendCommand, addCommand, disabled]);

  const pressOnly = useCallback((dir: Direction | null) => {
    setPressed({
      up: dir === 'up',
      down: dir === 'down',
      left: dir === 'left',
      right: dir === 'right',
    });
  }, []);

  const startDirection = useCallback(async (direction: Direction) => {
    // If switching, stop current first to avoid overlap on some motor drivers.
    if (activeDirRef.current && activeDirRef.current !== direction) {
      await send(MOVE_STOP);
    }
    // If already active, do nothing.
    if (activeDirRef.current === direction) return;

    await send(directionToCommand[direction]);
    activeDirRef.current = direction;
    pressOnly(direction);
  }, [send, pressOnly]);

  const stopAll = useCallback(async () => {
    if (activeDirRef.current !== null) {
      await send(MOVE_STOP);
    }
    activeDirRef.current = null;
    pressOnly(null);
  }, [send, pressOnly]);

  // --- Keyboard: WASD + Space (no Arrow keys; Camera owns those) ---
  const onKeyDown = useCallback((e: KeyboardEvent) => {
    const el = document.activeElement as HTMLElement | null;
    if (el && (el.isContentEditable || el.tagName === 'INPUT' || el.tagName === 'TEXTAREA')) return;

    const k = e.key.toLowerCase();
    if (k === ' ') {
      e.preventDefault();
      void stopAll(); // immediate hard stop
      return;
    }

    const dir = keyToDirection[k];
    if (!dir) return;
    if (e.repeat) { // ignore OS auto-repeat (we manage state ourselves)
      e.preventDefault();
      return;
    }
    e.preventDefault();
    void startDirection(dir);
  }, [startDirection, stopAll]);

  const onKeyUp = useCallback((e: KeyboardEvent) => {
    const k = e.key.toLowerCase();
    if (k === ' ') {
      e.preventDefault();
      return; // already stopped on keydown
    }
    const dir = keyToDirection[k];
    if (!dir) return;

    // Only stop if the released key matches the *currently active* direction.
    if (activeDirRef.current === dir) {
      e.preventDefault();
      void stopAll();
    }
  }, [stopAll]);

  // Safety guards
  useEffect(() => {
    const hardStop = () => void stopAll();
    window.addEventListener('keydown', onKeyDown, { passive: false });
    window.addEventListener('keyup', onKeyUp, { passive: false });
    window.addEventListener('blur', hardStop);
    const vis = () => { if (document.visibilityState !== 'visible') hardStop(); };
    document.addEventListener('visibilitychange', vis);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      window.removeEventListener('blur', hardStop);
      document.removeEventListener('visibilitychange', vis);
    };
  }, [onKeyDown, onKeyUp, stopAll]);

  // Unified pointer handling (mouse + touch + pen)
  const onPointerDown = (direction: Direction) => (e: React.PointerEvent) => {
    e.preventDefault();
    (e.currentTarget as HTMLElement).setPointerCapture?.(e.pointerId);
    void startDirection(direction);
  };
  const onPointerUp = (_direction: Direction) => (e: React.PointerEvent) => {
    e.preventDefault();
    void stopAll();
  };
  const onPointerCancel = (_direction: Direction) => (e: React.PointerEvent) => {
    e.preventDefault();
    void stopAll();
  };
  const onPointerLeave = (_direction: Direction) => (e: React.PointerEvent) => {
    e.preventDefault();
    void stopAll();
  };

  // Color mapping for car directions - always show colors (same as camera panel)
  const directionColors: Record<Direction, { normal: string; pressed: string }> = {
    up: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
    down: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
    left: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
    right: { normal: 'bg-sky-600', pressed: 'bg-sky-700 ring-sky-300' },
  };

  const buttonClass = (direction: Direction) => {
    const colors = directionColors[direction];
    const baseColor = pressed[direction] ? colors.pressed : colors.normal;
    return `p-4 m-1 rounded-lg text-white font-semibold select-none
     w-14 h-14 flex items-center justify-center outline-none
     transition-colors duration-100
     ${baseColor} ${
       disabled 
         ? 'cursor-not-allowed opacity-50' 
         : pressed[direction]
         ? 'ring-2'
         : 'hover:opacity-90'
     }`;
  };

  return (
    <div className={`flex flex-col items-center ${disabled ? 'opacity-75' : ''}`} onContextMenu={(e) => e.preventDefault()}>
      <div className="flex items-center gap-2 mb-2">
        <div className="text-lg font-bold">Car Control</div>
        <span className={`inline-block rounded-full ${status === 'connected' ? 'bg-emerald-500' : status === 'connecting' ? 'bg-slate-500' : 'bg-rose-500'}`} style={{ width: 8, height: 8 }} title={`Movement server: ${status}`} />
      </div>

      <button
        disabled={disabled}
        aria-label="Move Up (W)"
        aria-pressed={pressed.up}
        title={`Move Up (W)${disabled ? ' - Disconnected' : ''}`}
        className={buttonClass('up')}
        onPointerDown={disabled ? undefined : onPointerDown('up')}
        onPointerUp={disabled ? undefined : onPointerUp('up')}
        onPointerCancel={disabled ? undefined : onPointerCancel('up')}
        onPointerLeave={disabled ? undefined : onPointerLeave('up')}
      >
        W
      </button>

      <div className="flex space-x-2">
        <button
          disabled={disabled}
          aria-label="Move Left (A)"
          aria-pressed={pressed.left}
          title={`Move Left (A)${disabled ? ' - Disconnected' : ''}`}
          className={buttonClass('left')}
          onPointerDown={disabled ? undefined : onPointerDown('left')}
          onPointerUp={disabled ? undefined : onPointerUp('left')}
          onPointerCancel={disabled ? undefined : onPointerCancel('left')}
          onPointerLeave={disabled ? undefined : onPointerLeave('left')}
        >
          A
        </button>

        <button
          disabled={disabled}
          aria-label="Move Right (D)"
          aria-pressed={pressed.right}
          title={`Move Right (D)${disabled ? ' - Disconnected' : ''}`}
          className={buttonClass('right')}
          onPointerDown={disabled ? undefined : onPointerDown('right')}
          onPointerUp={disabled ? undefined : onPointerUp('right')}
          onPointerCancel={disabled ? undefined : onPointerCancel('right')}
          onPointerLeave={disabled ? undefined : onPointerLeave('right')}
        >
          D
        </button>
      </div>

      <button
        disabled={disabled}
        aria-label="Move Down (S)"
        aria-pressed={pressed.down}
        title={`Move Down (S)${disabled ? ' - Disconnected' : ''}`}
        className={buttonClass('down')}
        onPointerDown={disabled ? undefined : onPointerDown('down')}
        onPointerUp={disabled ? undefined : onPointerUp('down')}
        onPointerCancel={disabled ? undefined : onPointerCancel('down')}
        onPointerLeave={disabled ? undefined : onPointerLeave('down')}
      >
        S
      </button>

      {/* No explicit Stop button — Spacebar remains the emergency stop. */}
      <p className="text-xs text-zinc-400 mt-1">
        {disabled ? `Status: ${status}` : 'Tips: W/A/S/D to drive • Space to Stop'}
      </p>
    </div>
  );
};

export default CarControlPanel;
