/*
# File: /src/components/control/SpeedControl.tsx
# Summary:
Provides a user interface for controlling the robot's speed, LEDs, and buzzer.
Now also shows a small connection status dot (connected/connecting/disconnected) for the Movement WS.
*/

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { COMMAND } from '@/control_definitions';
import { useCommand } from '@/context/CommandContext';
import LedModal from '@/components/lighting/LedModal';
import { resolveWsUrl } from '@/utils/resolveWsUrl';

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

// Resolve movement WS from env (profile-aware)
const MOVEMENT_WS = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT');

// tiny status dot
function StatusDot({ status, title }: { status: ServerStatus; title: string }) {
  const color =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'connecting'
      ? 'bg-slate-500'
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

const SpeedControl: React.FC = () => {
  const { sendCommand } = useCommand();

  const [speed, setSpeed] = useState(0);
  const [isLedActive, setIsLedActive] = useState(false);
  const [isLedModalOpen, setIsLedModalOpen] = useState(false);

  // movement server status
  const [moveStatus, setMoveStatus] = useState<ServerStatus>('disconnected');
  const moveWsRef = useRef<WebSocket | null>(null);
  const retryRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const mounted = useRef(false);

  // --- Movement WS status monitor (separate, lightweight connection) ---
  useEffect(() => {
    mounted.current = true;
    if (!MOVEMENT_WS) {
      setMoveStatus('disconnected');
      return () => { mounted.current = false; };
    }

    const open = (attempt = 0) => {
      if (!mounted.current) return;
      setMoveStatus('connecting');

      let ws: WebSocket;
      try {
        ws = new WebSocket(MOVEMENT_WS);
      } catch {
        const backoff = Math.min(1000 * 2 ** attempt, 10000);
        retryRef.current = setTimeout(() => open(attempt + 1), backoff);
        return;
      }

      moveWsRef.current = ws;

      ws.onopen = () => {
        if (!mounted.current) return;
        setMoveStatus('connected');
      };

      ws.onerror = () => {
        // let onclose handle the state/backoff
      };

      ws.onclose = () => {
        if (!mounted.current) return;
        setMoveStatus('disconnected');
        const backoff = Math.min(1000 * 2 ** attempt, 10000);
        retryRef.current = setTimeout(() => open(attempt + 1), backoff);
      };
    };

    open(0);

    return () => {
      mounted.current = false;
      if (retryRef.current) clearTimeout(retryRef.current);
      retryRef.current = null;
      if (moveWsRef.current) {
        moveWsRef.current.onopen = null;
        moveWsRef.current.onclose = null;
        moveWsRef.current.onerror = null;
        try { moveWsRef.current.close(); } catch {}
        moveWsRef.current = null;
      }
    };
  }, []);

  // --- Controls ---
  const increaseSpeed = useCallback(() => {
    const newSpeed = Math.min(speed + 10, 100);
    setSpeed(newSpeed);
    sendCommand(`${COMMAND.INCREASE_SPEED}-${newSpeed}`);
  }, [speed, sendCommand]);

  const decreaseSpeed = useCallback(() => {
    const newSpeed = Math.max(speed - 10, 0);
    setSpeed(newSpeed);
    sendCommand(`${COMMAND.DECREASE_SPEED}-${newSpeed}`);
  }, [speed, sendCommand]);

  const emergencyStop = useCallback(() => {
    setSpeed(0);
    sendCommand(COMMAND.STOP);
  }, [sendCommand]);

  const toggleLed = useCallback(() => {
    setIsLedActive((prev) => !prev);
    setIsLedModalOpen(true);
  }, []);

  const activateBuzzer = useCallback(() => {
    sendCommand(COMMAND.CMD_BUZZER);
    setTimeout(() => sendCommand(COMMAND.CMD_BUZZER_STOP), 5000);
  }, [sendCommand]);

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key.toLowerCase()) {
        case 'p': increaseSpeed(); break;
        case 'o': decreaseSpeed(); break;
        case ' ': emergencyStop(); break;
        case 'i': toggleLed(); break;
        case '0': activateBuzzer(); break;
        default: break;
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [increaseSpeed, decreaseSpeed, emergencyStop, toggleLed, activateBuzzer]);

  const moveTitle =
    `Movement server: ${moveStatus[0].toUpperCase()}${moveStatus.slice(1)}`;

  return (
    <div className="bg-gray-800 text-white p-4 rounded-md shadow-md w-full max-w-sm flex flex-col items-center">
      {/* Header with status dot */}
      <div className="w-full flex items-center justify-between mb-4">
        <h2 className="text-lg font-bold">Speed & Light Control</h2>
        <StatusDot status={moveStatus} title={moveTitle} />
      </div>

      {/* Speed Progress Bar */}
      <div className="w-full mb-4">
        <label className="block text-sm font-semibold mb-2">Speed:</label>
        <div className="w-full bg-gray-300 rounded h-4 relative">
          <div
            className={`h-full rounded ${
              speed <= 20 ? 'bg-red-500' : speed <= 60 ? 'bg-yellow-500' : 'bg-green-500'
            }`}
            style={{ width: `${speed}%` }}
          />
          <div className="absolute inset-0 flex justify-center items-center text-white text-sm font-bold">
            {speed}%
          </div>
        </div>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-2 gap-3 w-full">
        <button className="bg-blue-600 text-white py-2 rounded hover:bg-blue-700" onClick={decreaseSpeed}>
          O (Brake)
        </button>
        <button className="bg-blue-600 text-white py-2 rounded hover:bg-blue-700" onClick={increaseSpeed}>
          P (Gas)
        </button>
        <button className="bg-red-600 text-white py-2 rounded hover:bg-red-700 col-span-2" onClick={emergencyStop}>
          Space (Stop)
        </button>
        <button className="bg-yellow-600 text-white py-2 rounded hover:bg-yellow-700 col-span-2" onClick={toggleLed}>
          I (LED)
        </button>
        <button className="bg-purple-600 text-white py-2 rounded hover:bg-purple-700 col-span-2" onClick={activateBuzzer}>
          0 (Buzzer)
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
