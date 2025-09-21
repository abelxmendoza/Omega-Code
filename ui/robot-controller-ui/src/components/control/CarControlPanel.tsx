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
  const { sendCommand, addCommand } = useCommand();

  const [pressed, setPressed] = useState<Record<Direction, boolean>>({
    up: false, down: false, left: false, right: false,
  });

  const activeDirRef = useRef<Direction | null>(null);

  const send = useCallback(async (cmd: string) => {
    try {
      await sendCommand(cmd);
      addCommand(`Command Sent: ${cmd}`);
    } catch (e: any) {
      addCommand(`Command Error: ${cmd} — ${e?.message ?? e}`);
    }
  }, [sendCommand, addCommand]);

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

  const btnBase =
    'select-none p-4 m-1 rounded-lg text-white font-semibold transition-all ' +
    'duration-100 min-w-[3.25rem] min-h-[3.25rem] outline-none focus:ring-2 ' +
    'active:scale-[0.98]';

  // Match Camera panel active color/ring
  const buttonClass = (direction: Direction) =>
    `${btnBase} ${
      pressed[direction]
        ? 'bg-emerald-600 ring-emerald-300'
        : 'bg-gray-800 hover:bg-gray-700'
    }`;

  return (
    <div className="flex flex-col items-center gap-2" onContextMenu={(e) => e.preventDefault()}>
      <h2 className="text-lg font-bold">Car Control</h2>

      <button
        aria-label="Move Up (W)"
        aria-pressed={pressed.up}
        title="Move Up (W)"
        className={buttonClass('up')}
        onPointerDown={onPointerDown('up')}
        onPointerUp={onPointerUp('up')}
        onPointerCancel={onPointerCancel('up')}
        onPointerLeave={onPointerLeave('up')}
      >
        W
      </button>

      <div className="flex gap-2">
        <button
          aria-label="Move Left (A)"
          aria-pressed={pressed.left}
          title="Move Left (A)"
          className={buttonClass('left')}
          onPointerDown={onPointerDown('left')}
          onPointerUp={onPointerUp('left')}
          onPointerCancel={onPointerCancel('left')}
          onPointerLeave={onPointerLeave('left')}
        >
          A
        </button>

        <button
          aria-label="Move Right (D)"
          aria-pressed={pressed.right}
          title="Move Right (D)"
          className={buttonClass('right')}
          onPointerDown={onPointerDown('right')}
          onPointerUp={onPointerUp('right')}
          onPointerCancel={onPointerCancel('right')}
          onPointerLeave={onPointerLeave('right')}
        >
          D
        </button>
      </div>

      <button
        aria-label="Move Down (S)"
        aria-pressed={pressed.down}
        title="Move Down (S)"
        className={buttonClass('down')}
        onPointerDown={onPointerDown('down')}
        onPointerUp={onPointerUp('down')}
        onPointerCancel={onPointerCancel('down')}
        onPointerLeave={onPointerLeave('down')}
      >
        S
      </button>

      {/* No explicit Stop button — Spacebar remains the emergency stop. */}
      <p className="text-xs text-zinc-400 mt-1">Tips: W/A/S/D to drive • Space to Stop</p>
    </div>
  );
};

export default CarControlPanel;
