/**
 * usePiGamepad
 * Polls GET /gamepad/status on the FastAPI backend (port 8000) to detect
 * whether a physical gamepad is connected to the Raspberry Pi via evdev.
 * This covers the case where the controller is plugged into the Pi rather
 * than the browser machine (Web Gamepad API can't see Pi-local devices).
 */

import { useState, useEffect, useRef } from 'react';

export interface PiGamepadState {
  connected: boolean;
  name: string;
}

const POLL_INTERVAL_MS = 10_000; // gamepad presence doesn't need sub-second precision

export function usePiGamepad(apiBase: string): PiGamepadState {
  const [state, setState] = useState<PiGamepadState>({ connected: false, name: '' });
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    const base = (apiBase || '').replace(/\/$/, '') || 'http://localhost:8000';
    const url = `${base}/gamepad/status`;

    const poll = async () => {
      try {
        const res = await fetch(url, { signal: AbortSignal.timeout(2000) });
        if (!res.ok) return;
        const data = await res.json();
        setState({
          connected: Boolean(data.connected),
          name: data.name ?? '',
        });
      } catch {
        // backend unreachable — don't flip state, keep last known
      }
    };

    poll(); // immediate first poll
    timerRef.current = setInterval(poll, POLL_INTERVAL_MS);

    return () => {
      if (timerRef.current) clearInterval(timerRef.current);
    };
  }, [apiBase]);

  return state;
}
