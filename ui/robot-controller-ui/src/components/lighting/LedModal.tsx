/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/lighting/LedModal.tsx
# Summary:
Modal interface for controlling LED lighting on the robot. Supports single and rainbow modes,
power toggle, pattern selection, brightness control, and interval configuration for dynamic patterns.
Sends configuration to the backend via WebSocket.
Includes a live connection indicator (connecting/connected/disconnected) with a heartbeat and
layout tweaks so the status never overlaps the Close button. Adds safe cleanup + auto-reconnect.
*/

import React, { useState, useEffect, useRef } from 'react';
import { createPortal } from 'react-dom';
import type { ColorResult } from 'react-color';
import { SketchPicker } from 'react-color';
import { connectLightingWs } from '../../utils/connectLightingWs';

// Updated to match backend capabilities
const LIGHTING_MODES = ['single', 'rainbow', 'dual'] as const;
const LIGHTING_PATTERNS = ['static', 'pulse', 'blink', 'fade', 'chase', 'rainbow', 'lightshow', 'music'] as const;

type LightingMode = (typeof LIGHTING_MODES)[number];
type LightingPattern = (typeof LIGHTING_PATTERNS)[number];

interface LedModalProps {
  isOpen: boolean;
  onClose: () => void;
}

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

const LedModal: React.FC<LedModalProps> = ({ isOpen, onClose }) => {
  const [ledOn, setLedOn] = useState(true);
  const [color1, setColor1] = useState('#ffffff');
  const [mode, setMode] = useState<LightingMode>(LIGHTING_MODES[0]);
  const [pattern, setPattern] = useState<LightingPattern>(LIGHTING_PATTERNS[0]);
  const [intervalMs, setIntervalMs] = useState(1000);
  const [brightness, setBrightness] = useState(100); // 0–100%

  // --- connection status & heartbeat state ---
  const [serverStatus, setServerStatus] = useState<ServerStatus>('disconnected');
  const [latencyMs, setLatencyMs] = useState<number | null>(null);

  const ws = useRef<WebSocket | null>(null);
  const heartbeatTimer = useRef<ReturnType<typeof setInterval> | null>(null);
  const heartbeatInFlightAt = useRef<number | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const mounted = useRef(false);

  // Helpers
  const wsOpen = () => ws.current && ws.current.readyState === WebSocket.OPEN;

  const clearHeartbeat = () => {
    if (heartbeatTimer.current) {
      clearInterval(heartbeatTimer.current);
      heartbeatTimer.current = null;
    }
    heartbeatInFlightAt.current = null;
  };

  const startHeartbeat = () => {
    clearHeartbeat();
    heartbeatTimer.current = setInterval(() => {
      if (!wsOpen()) {
        setServerStatus('disconnected');
        return;
      }
      try {
        heartbeatInFlightAt.current = performance.now();
        ws.current!.send(JSON.stringify({ type: 'ping', ts: Date.now() }));
        // simple timeout if no pong
        setTimeout(() => {
          if (heartbeatInFlightAt.current) {
            // no pong received within window; mark degraded latency
            heartbeatInFlightAt.current = null;
            setLatencyMs(null);
          }
        }, 6000);
      } catch (e) {
        console.warn('Heartbeat send failed:', e);
        setServerStatus('disconnected');
      }
    }, 10000);
  };

  // Connection lifecycle w/ auto-reconnect while modal is open
  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);

  useEffect(() => {
    if (!isOpen) {
      // Cleanup when modal closes
      clearHeartbeat();
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
      if (ws.current) {
        // Avoid invoking handlers post-close
        ws.current.onopen = null;
        ws.current.onclose = null;
        ws.current.onerror = null;
        ws.current.onmessage = null;
        try {
          ws.current.close();
        } catch {}
        ws.current = null;
      }
      setLatencyMs(null);
      setServerStatus('disconnected');
      return;
    }

    let cancelled = false;

    const openWs = async (attempt = 0) => {
      if (cancelled) return;

      setServerStatus('connecting');
      try {
        console.log('[LedModal] Attempting to connect to lighting WebSocket...');
        const { lightingCandidates } = await import('../../utils/connectLightingWs');
        const candidates = lightingCandidates();
        console.log('[LedModal] WebSocket candidates:', candidates);
        const wsObj = await connectLightingWs();
        console.log('[LedModal] Successfully connected to lighting WebSocket');
        if (cancelled) {
          try { wsObj.close(); } catch {}
          return;
        }
        ws.current = wsObj;

        wsObj.onopen = () => {
          if (!mounted.current) return;
          setServerStatus('connected');
          setLatencyMs(null);
          startHeartbeat();
        };

        wsObj.onclose = () => {
          if (!mounted.current) return;
          setServerStatus('disconnected');
          clearHeartbeat();
          // schedule reconnect if still open
          if (isOpen) {
            const backoff = Math.min(1000 * Math.pow(2, attempt), 10000); // 1s → 10s
            if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
            reconnectTimer.current = setTimeout(() => openWs(attempt + 1), backoff);
          }
        };

        wsObj.onerror = () => {
          if (!mounted.current) return;
          // Let onclose handle the reconnect
          try { wsObj.close(); } catch {}
        };

        wsObj.onmessage = (event) => {
          if (!mounted.current) return;
          try {
            const data = JSON.parse(event.data);
            if (data?.status === 'connected' && data?.service === 'lighting') {
              setServerStatus('connected');
            }
            if (data?.type === 'pong' && typeof data?.ts === 'number') {
              const end = performance.now();
              const start = heartbeatInFlightAt.current ?? end;
              setLatencyMs(Math.max(0, Math.round(end - start)));
              heartbeatInFlightAt.current = null;
            }
          } catch {
            // ignore non-JSON messages
          }
        };
      } catch (err) {
        if (!mounted.current) return;
        console.error('[LedModal] Lighting WS failed to connect:', err);
        setServerStatus('disconnected');
        // retry
        const backoff = Math.min(1000 * Math.pow(2, attempt), 10000);
        if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
        reconnectTimer.current = setTimeout(() => openWs(attempt + 1), backoff);
      }
    };

    openWs(0);

    return () => {
      cancelled = true;
      clearHeartbeat();
      if (reconnectTimer.current) {
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
      }
      if (ws.current) {
        ws.current.onopen = null;
        ws.current.onclose = null;
        ws.current.onerror = null;
        ws.current.onmessage = null;
        try { ws.current.close(); } catch {}
        ws.current = null;
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isOpen]);

  // Handlers
  const handleColor1Change = (color: ColorResult) => setColor1(color.hex);
  const handleModeChange = (e: React.ChangeEvent<HTMLSelectElement>) =>
    setMode(e.target.value as LightingMode);
  const handlePatternChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const nextPattern = e.target.value as LightingPattern;
    setPattern(nextPattern);

    if (nextPattern === 'music' && intervalMs > 250) {
      setIntervalMs(120);
    }
  };
  const handleIntervalChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    setIntervalMs(Number.isFinite(value) ? Math.max(100, Math.floor(value)) : 100);
  };
  const handleBrightnessChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    if (Number.isFinite(value)) setBrightness(Math.min(100, Math.max(0, Math.floor(value))));
  };

  const send = (payload: Record<string, unknown>) => {
    if (!wsOpen()) {
      console.error('Lighting WS not open.');
      return;
    }
    ws.current!.send(JSON.stringify(payload));
  };

  const handleTogglePower = () => {
    const newState = !ledOn;
    setLedOn(newState);

    if (!wsOpen()) {
      console.error('Lighting WS not open.');
      return;
    }

    if (!newState) {
      // Send complete command with pattern='off' to turn off LEDs
      send({
        color: '#000000',
        mode: 'single',
        pattern: 'off',
        interval: 0,
        brightness: 0,
      });
      console.log('LED OFF command sent!');
    } else {
      // Send complete command with current settings to turn on LEDs
      send({
        color: color1,
        mode: mode,
        pattern: pattern,
        interval: pattern !== 'static' ? intervalMs : 0,
        brightness: brightness / 100, // Convert 0-100 to 0-1
      });
      console.log('LED ON command sent with color:', color1, 'mode:', mode, 'pattern:', pattern);
    }
  };

  const handleApply = () => {
    if (!ledOn) {
      console.log('LED is off, skipping apply.');
      return;
    }
    if (!wsOpen()) {
      console.error('Lighting WS not open.');
      return;
    }
    
    // Validate interval for dynamic patterns
    if (pattern !== 'static' && intervalMs <= 0) {
      console.error('Interval must be greater than 0 for dynamic patterns.');
      return;
    }
    
    // Send complete command payload matching backend LightingCommand struct
    // Backend expects: { color, mode, pattern, interval, brightness }
    const commandData = {
      color: color1, // Hex string like "#ff0000"
      mode: mode,
      pattern: pattern,
      interval: pattern !== 'static' ? intervalMs : 0,
      brightness: brightness / 100, // Convert 0-100% to 0-1.0
    };
    send(commandData);
    console.log('LED settings applied:', commandData);
  };

  // STATUS PILL COLORS
  const statusColor =
    serverStatus === 'connected'
      ? 'bg-emerald-500'
      : serverStatus === 'connecting'
      ? 'bg-slate-600'
      : 'bg-rose-600';

  const statusLabel =
    serverStatus === 'connected'
      ? latencyMs != null
        ? `Connected • ${latencyMs}ms`
        : 'Connected'
      : serverStatus === 'connecting'
      ? 'Connecting…'
      : 'Disconnected';

  const intervalConfig =
    pattern === 'music'
      ? { min: 50, step: 10 }
      : { min: 100, step: 50 };

  if (!isOpen) return null;

  return createPortal(
    <div
      className="fixed inset-0 bg-gray-800/75 backdrop-blur-sm flex justify-center items-center z-[9999] p-4"
      style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0 }}
      role="dialog"
      aria-modal="true"
      aria-labelledby="led-config-title"
    >
      <div className="bg-gray-900 rounded-lg p-8 w-full max-w-2xl relative text-white shadow-2xl border-2 border-purple-500/50 animate-in zoom-in-95 duration-200" style={{ position: 'relative', zIndex: 10000 }}>
        {/* Close Button (absolute) */}
        <button
          type="button"
          className="absolute top-3 right-3 text-white bg-red-500 hover:bg-red-600 p-2 rounded focus:outline-none focus:ring-2 focus:ring-red-400 z-[10001]"
          style={{ position: 'absolute', zIndex: 10001 }}
          onClick={onClose}
          aria-label="Close LED configuration"
        >
          Close
        </button>

        {/* Title + Connection Status */}
        <div className="flex items-center justify-between border-b-2 border-green-400 pb-2 mb-4 pr-20">
          <h2 id="led-config-title" className="text-lg font-bold text-green-400">
            LED Configuration
          </h2>
          <div
            className={`flex items-center gap-2 px-2 py-1 rounded-full text-xs ${statusColor}`}
            title="Lighting server connection status"
            aria-live="polite"
          >
            <span className="inline-block w-2 h-2 rounded-full bg-white/90" />
            <span className="font-semibold">{statusLabel}</span>
          </div>
        </div>

        {/* LED Power Toggle */}
        <div className="mt-2 mb-4 flex justify-between items-center">
          <span className="text-green-300 font-semibold">
            LED Status: {ledOn ? 'On' : 'Off'}
          </span>
          <button
            onClick={handleTogglePower}
            disabled={serverStatus !== 'connected'}
            className={`px-4 py-2 rounded text-white focus:outline-none focus:ring-2 ${
              serverStatus !== 'connected'
                ? 'bg-gray-600 cursor-not-allowed'
                : ledOn
                ? 'bg-red-500 hover:bg-red-600 focus:ring-red-400'
                : 'bg-green-500 hover:bg-green-600 focus:ring-green-400'
            }`}
          >
            {serverStatus !== 'connected'
              ? 'Server Unavailable'
              : `Turn ${ledOn ? 'Off' : 'On'}`}
          </button>
        </div>

        <fieldset
          disabled={!ledOn || serverStatus !== 'connected'}
          className={ledOn && serverStatus === 'connected' ? '' : 'opacity-50'}
        >
          {/* Color Picker 1 */}
          <label className="block text-green-300 font-semibold mb-1">Primary Color:</label>
          <SketchPicker color={color1} onChange={handleColor1Change} />

          {/* Mode Selector */}
          <div className="mt-4">
            <label htmlFor="mode" className="block text-green-300 font-semibold">
              Mode:
            </label>
            <select
              id="mode"
              value={mode}
              onChange={handleModeChange}
              className="w-full bg-gray-800 text-white p-2 rounded mt-1"
            >
              {LIGHTING_MODES.map((m) => (
                <option key={m} value={m}>
                  {m.charAt(0).toUpperCase() + m.slice(1)}
                </option>
              ))}
            </select>
          </div>

          {/* Pattern Selector */}
          <div className="mt-4">
            <label htmlFor="pattern" className="block text-green-300 font-semibold">
              Pattern:
            </label>
            <select
              id="pattern"
              value={pattern}
              onChange={handlePatternChange}
              className="w-full bg-gray-800 text-white p-2 rounded mt-1"
            >
              {LIGHTING_PATTERNS.map((p) => (
                <option key={p} value={p}>
                  {p.charAt(0).toUpperCase() + p.slice(1)}
                </option>
              ))}
            </select>
            {pattern === 'music' && (
              <p className="mt-2 text-sm text-green-200">
                Music mode listens to the default microphone when available and generates a
                beat-reactive light show. Reduce the interval for faster reaction times.
              </p>
            )}
            {pattern === 'fade' && (
              <p className="mt-2 text-sm text-green-200">
                Fade creates smooth color transitions. Adjust interval to control fade speed.
              </p>
            )}
            {pattern === 'chase' && (
              <p className="mt-2 text-sm text-green-200">
                Chase creates a moving light effect across the LED strip. Lower interval = faster chase.
              </p>
            )}
            {pattern === 'lightshow' && (
              <p className="mt-2 text-sm text-green-200">
                Lightshow creates a multi-stage animated display with rotating color bands and sparkles.
              </p>
            )}
            {pattern === 'rainbow' && (
              <p className="mt-2 text-sm text-green-200">
                Rainbow displays a full spectrum sweep across all LEDs. Interval controls animation speed.
              </p>
            )}
          </div>

          {/* Interval Input */}
          {pattern !== 'static' && (
            <div className="mt-4">
              <label htmlFor="interval" className="block text-green-300 font-semibold">
                Interval (ms):
              </label>
              <input
                id="interval"
                type="number"
                value={intervalMs}
                onChange={handleIntervalChange}
                className="w-full bg-gray-800 text-white p-2 rounded mt-1"
                min={intervalConfig.min}
                step={intervalConfig.step}
              />
            </div>
          )}

          {/* Brightness Slider */}
          <div className="mt-4">
            <label htmlFor="brightness" className="block text-green-300 font-semibold">
              Brightness (%):
            </label>
            <input
              id="brightness"
              type="range"
              min={0}
              max={100}
              value={brightness}
              onChange={handleBrightnessChange}
              className="w-full"
            />
          </div>
        </fieldset>

        {/* Apply Button */}
        <button
          onClick={handleApply}
          disabled={serverStatus !== 'connected' || !ledOn}
          className={`text-white p-3 rounded mt-6 w-full focus:outline-none focus:ring-2 transition-all ${
            serverStatus !== 'connected' || !ledOn
              ? 'bg-gray-600 cursor-not-allowed'
              : 'bg-purple-600 hover:bg-purple-700 focus:ring-purple-400 hover:shadow-lg'
          }`}
        >
          {serverStatus !== 'connected' ? 'Server Unavailable' : 'Apply Settings'}
        </button>
      </div>
    </div>,
    document.body
  );
};

export default LedModal;
