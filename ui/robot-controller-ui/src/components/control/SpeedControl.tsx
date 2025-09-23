/*
# File: /src/components/control/SpeedControl.tsx
# Summary:
Clean speed + LED + Horn (hold) control using CommandContext (single WS).
- P = Gas (+10%), O = Brake (-10%), Space = Stop
- 0 / Numpad0 = Horn (hold to buzz, release to stop)
- Uses set-speed with PWM value (0–4095) derived from 0–100% UI
- Disables controls when disconnected; shows connection dot + optional latency
*/

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';
import LedModal from '@/components/lighting/LedModal';

type ServerStatus = 'connecting' | 'connected' | 'disconnected' | 'error';

// Small connection status dot
function StatusDot({ status, title }: { status: ServerStatus; title: string }) {
  const color =
    status === 'connected' ? 'bg-emerald-500'
    : status === 'connecting' ? 'bg-slate-500'
    : status === 'error' ? 'bg-red-600'
    : 'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const PWM_MAX = 4095; // keep in sync with the movement server

// Resolve command names with safe fallbacks (keeps UI resilient to backend/const changes)
const CMD_SET_SPEED   = (COMMAND as any)?.SET_SPEED       || 'set-speed';
const CMD_STOP        = (COMMAND as any)?.STOP            || 'stop';
const CMD_BUZZ_ON     = (COMMAND as any)?.CMD_BUZZER      || 'buzz';
const CMD_BUZZ_OFF    = (COMMAND as any)?.CMD_BUZZER_STOP || 'buzz-stop';

const SpeedControl: React.FC = () => {
  const { sendCommand, status, latencyMs } = useCommand();

  // UI speed percent (0–100). Sent to server as PWM (0–4095).
  const [speedPct, setSpeedPct] = useState(0);
  const [isLedModalOpen, setIsLedModalOpen] = useState(false);

  const disabled = status !== 'connected';
  const moveTitle = `Movement server: ${status[0].toUpperCase()}${status.slice(1)}${
    latencyMs != null ? ` • ~${latencyMs} ms` : ''
  }`;

  // Debounce for slider -> fewer WS messages
  const debounceRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  // Horn state (avoid repeat spam, ensure clean stop)
  const hornHeldRef = useRef(false);

  // --- Speed helpers ---------------------------------------------------------
  const sendSetSpeedPct = useCallback((pct: number) => {
    const clampedPct = clamp(Math.round(pct), 0, 100);
    const pwm = Math.round((clampedPct / 100) * PWM_MAX);
    setSpeedPct(clampedPct);
    try {
      sendCommand(CMD_SET_SPEED, { value: pwm }); // server expects "value"
    } catch {
      // CommandContext logs errors; keep UI responsive
    }
  }, [sendCommand]);

  const increaseSpeed = useCallback(() => {
    if (disabled) return;
    sendSetSpeedPct(speedPct + 10);
  }, [disabled, speedPct, sendSetSpeedPct]);

  const decreaseSpeed = useCallback(() => {
    if (disabled) return;
    sendSetSpeedPct(speedPct - 10);
  }, [disabled, speedPct, sendSetSpeedPct]);

  const emergencyStop = useCallback(() => {
    if (disabled) return;
    // Stop motors and explicitly set speed to 0 (keeps UI + backend in sync)
    sendSetSpeedPct(0);
    try { sendCommand(CMD_STOP); } catch {}
  }, [disabled, sendSetSpeedPct, sendCommand]);

  const onSliderChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const next = Number(e.target.value);
    setSpeedPct(next); // instant UI feedback
    if (debounceRef.current) clearTimeout(debounceRef.current);
    debounceRef.current = setTimeout(() => {
      if (!disabled) sendSetSpeedPct(next);
    }, 120);
  }, [disabled, sendSetSpeedPct]);

  // --- LED -------------------------------------------------------------------
  const openLed = useCallback(() => setIsLedModalOpen(true), []);

  // --- Horn (hold-to-buzz) ---------------------------------------------------
  const hornDown = useCallback(() => {
    if (disabled || hornHeldRef.current) return;
    hornHeldRef.current = true;
    try { sendCommand(CMD_BUZZ_ON); } catch {}
  }, [disabled, sendCommand]);

  const hornUp = useCallback(() => {
    if (!hornHeldRef.current) return;
    hornHeldRef.current = false;
    try { sendCommand(CMD_BUZZ_OFF); } catch {}
  }, [sendCommand]);

  // Stop horn on disconnect just in case
  useEffect(() => {
    if (status !== 'connected') hornUp();
  }, [status, hornUp]);

  // Helper: treat text-like inputs as typing; allow Space/0 when focused on non-typing inputs (e.g., range)
  const isTypingField = (el: Element | null): boolean => {
    if (!el) return false;
    const node = el as HTMLElement;
    if (node.isContentEditable) return true;
    const tag = node.tagName;
    if (tag !== 'INPUT' && tag !== 'TEXTAREA') return false;
    const type = (node as HTMLInputElement).type?.toLowerCase?.() || '';
    return ['text','search','email','password','url','tel','number'].includes(type);
  };

  // --- Keyboard shortcuts ----------------------------------------------------
  // P = gas, O = brake, Space = stop, I = LED, 0/Numpad0 = Horn (hold)
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      const targetIsTyping = isTypingField(document.activeElement);
      const isSpace = event.key === ' ';
      const isHorn = event.key === '0' || event.code === 'Numpad0';

      // Respect typing fields except for Space (stop) and Horn (0/Numpad0)
      if (targetIsTyping && !isSpace && !isHorn) return;

      if (isSpace || isHorn) event.preventDefault(); // avoid scroll / native quirks
      // Ignore OS key-repeat for horn; we send once on first keydown
      if (event.repeat && isHorn) return;

      switch (event.key.toLowerCase()) {
        case 'p': increaseSpeed(); break;
        case 'o': decreaseSpeed(); break;
        case ' ': emergencyStop(); break;
        case 'i': openLed(); break;
        case '0': hornDown(); break;
        default:
          if (event.code === 'Numpad0') hornDown();
          break;
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      if (event.key === '0' || event.code === 'Numpad0') hornUp();
    };

    const bail = () => hornUp();
    const onVis = () => { if (document.visibilityState !== 'visible') bail(); };

    window.addEventListener('keydown', handleKeyDown, { passive: false });
    window.addEventListener('keyup', handleKeyUp, { passive: false });
    window.addEventListener('blur', bail);
    document.addEventListener('visibilitychange', onVis);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      window.removeEventListener('blur', bail);
      document.removeEventListener('visibilitychange', onVis);
      hornUp(); // safety
      if (debounceRef.current) clearTimeout(debounceRef.current);
    };
  }, [increaseSpeed, decreaseSpeed, emergencyStop, openLed, hornDown, hornUp]);

  // Pointer handlers for horn button (mouse + touch)
  const hornPointerHandlers = {
    onPointerDown: (e: React.PointerEvent) => { e.preventDefault(); hornDown(); },
    onPointerUp:   (e: React.PointerEvent) => { e.preventDefault(); hornUp();   },
    onPointerCancel:(e: React.PointerEvent)=> { e.preventDefault(); hornUp();   },
    onPointerLeave:(e: React.PointerEvent) => { e.preventDefault(); hornUp();   },
  };

  return (
    <div className="bg-gray-800 text-white p-4 rounded-md shadow-md w-full max-w-sm flex flex-col items-center">
      {/* Header with status dot */}
      <div className="w-full flex items-center justify-between mb-4">
        <h2 className="text-lg font-bold">Speed, Light & Horn</h2>
        <StatusDot status={status} title={moveTitle} />
      </div>

      {/* Speed Progress + Slider */}
      <div className="w-full mb-3">
        <label className="block text-sm font-semibold mb-2">
          Speed: {speedPct}% {latencyMs != null ? <span className="text-xs text-zinc-300">(~{latencyMs}ms)</span> : null}
        </label>
        <div className="w-full bg-gray-300 rounded h-3 mb-2 relative" aria-hidden="true">
          <div
            className={`h-full rounded ${speedPct <= 20 ? 'bg-red-500' : speedPct <= 60 ? 'bg-yellow-500' : 'bg-green-500'}`}
            style={{ width: `${speedPct}%` }}
          />
        </div>
        <input
          type="range"
          min={0}
          max={100}
          step={1}
          value={speedPct}
          onChange={onSliderChange}
          disabled={disabled}
          className="w-full accent-green-500"
          aria-label="Set speed percent"
        />
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-3 w-full">
        <button
          className={`py-2 rounded text-white transition-colors ${
            disabled ? 'bg-blue-900 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
          }`}
          onClick={decreaseSpeed}
          disabled={disabled}
          title="O (Brake)"
          aria-label="Decrease speed (O)"
        >
          O (Brake)
        </button>

        <button
          className={`py-2 rounded text-white transition-colors ${
            disabled ? 'bg-blue-900 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
          }`}
          onClick={increaseSpeed}
          disabled={disabled}
          title="P (Gas)"
          aria-label="Increase speed (P)"
        >
          P (Gas)
        </button>

        <button
          className={`col-span-2 py-2 rounded text-white transition-colors ${
            disabled ? 'bg-red-900 cursor-not-allowed' : 'bg-red-600 hover:bg-red-700'
          }`}
          onClick={emergencyStop}
          disabled={disabled}
          title="Space (Stop)"
          aria-label="Emergency stop (Space)"
        >
          Space (Stop)
        </button>

        <button
          className="col-span-2 py-2 rounded text-white bg-yellow-600 hover:bg-yellow-700 transition-colors"
          onClick={openLed}
          title="I (LED)"
          aria-label="Open LED controls (I)"
        >
          I (LED)
        </button>

        <button
          className={`col-span-2 py-2 rounded text-white select-none transition-colors ${
            disabled ? 'bg-purple-900 cursor-not-allowed' : 'bg-purple-600 hover:bg-purple-700'
          }`}
          {...hornPointerHandlers}
          disabled={disabled}
          title="Hold 0 key or press-and-hold"
          aria-label="Horn (hold 0)"
        >
          Horn (hold 0)
        </button>
      </div>

      {/* LED Modal */}
      {isLedModalOpen && (
        <LedModal isOpen={isLedModalOpen} onClose={() => setIsLedModalOpen(false)} />
      )}
    </div>
  );
};

export default SpeedControl;
