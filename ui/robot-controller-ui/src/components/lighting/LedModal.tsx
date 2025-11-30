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
import { Switch } from '../ui/switch';

// Updated to match backend capabilities - optimized with cool patterns
const LIGHTING_MODES = ['single', 'rainbow', 'dual'] as const;

// Patterns organized by mode
const SINGLE_MODE_PATTERNS = [
  'static',      // Solid color
  'pulse',       // Fade in/out
  'blink',       // On/off blinking
  'fade',        // Smooth transitions
  'chase',       // Moving chase effect
  'breathing',   // Smooth breathing pulse
  'aurora',      // Flowing northern lights
  'matrix',      // Matrix rain effect
  'fire',        // Flickering fire effect
  'lightshow',   // Multi-stage animation
  'music',       // Audio reactive
  'rave',        // Energetic dancing lights
] as const;

const DUAL_MODE_PATTERNS = [
  'static',      // Alternating colors
  'blink',       // Blink between two colors
  'fade',        // Fade between two colors
  'chase',       // Chase with two colors
  'pulse',       // Pulse between two colors
] as const;

const RAINBOW_MODE_PATTERNS = [
  'rainbow',     // Spectrum sweep
  'lightshow',   // Multi-stage animation
  'rave',        // Energetic dancing lights
  'aurora',      // Flowing northern lights
] as const;

// All patterns combined (for type definition)
const LIGHTING_PATTERNS = [
  ...SINGLE_MODE_PATTERNS,
  ...DUAL_MODE_PATTERNS,
  ...RAINBOW_MODE_PATTERNS,
] as const;

type LightingMode = (typeof LIGHTING_MODES)[number];
type LightingPattern = (typeof LIGHTING_PATTERNS)[number];

interface LedModalProps {
  isOpen: boolean;
  onClose: () => void;
}

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

const LedModal: React.FC<LedModalProps> = ({ isOpen, onClose }) => {
  const [ledOn, setLedOn] = useState(false);
  const [color1, setColor1] = useState('#ffffff');
  const [color2, setColor2] = useState('#000000');
  const [mode, setMode] = useState<LightingMode>(LIGHTING_MODES[0]);
  const [pattern, setPattern] = useState<LightingPattern>('static');
  const [intervalMs, setIntervalMs] = useState(1000);
  const [brightness, setBrightness] = useState(35); // Default 35% brightness (0–100%)

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

        // Check if WebSocket is already open (connectLightingWs waits for OPEN)
        if (wsObj.readyState === WebSocket.OPEN) {
          console.log('[LedModal] ✅ WebSocket already open, setting status to connected');
          setServerStatus('connected');
          setLatencyMs(null);
          startHeartbeat();
        }

        wsObj.onopen = () => {
          if (!mounted.current) return;
          console.log('[LedModal] ✅ WebSocket opened successfully');
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
            console.log('[LedModal] 📥 Received message:', data);
            if (data?.status === 'connected' && data?.service === 'lighting') {
              console.log('[LedModal] ✅ Welcome message received, status set to connected');
              setServerStatus('connected');
            }
            if (data?.type === 'pong' && typeof data?.ts === 'number') {
              const end = performance.now();
              const start = heartbeatInFlightAt.current ?? end;
              setLatencyMs(Math.max(0, Math.round(end - start)));
              heartbeatInFlightAt.current = null;
            }
            if (data?.type === 'lighting_result') {
              console.log('[LedModal] 📨 Lighting result:', data);
              if (data?.ok === false) {
                console.error('[LedModal] ❌ Lighting command failed:', data?.error);
                // Revert toggle state if command failed
                if (data?.error && typeof data.error === 'string') {
                  if (data.error.includes('off') || data.error.includes('OFF')) {
                    setLedOn(true); // Revert to ON if OFF command failed
                  }
                }
              } else {
                console.log('[LedModal] ✅ Lighting command succeeded');
              }
            }
            if (data?.type === 'error') {
              console.error('[LedModal] ❌ Server error:', data?.error);
              // Don't revert state on server errors - let user retry
            }
          } catch (error) {
            console.error('[LedModal] ❌ Failed to parse message:', error, 'Raw:', event.data);
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

  // Helper to auto-apply changes when LED is on
  const autoApplyIfOn = () => {
    if (ledOn && wsOpen() && serverStatus === 'connected') {
      // Small delay to batch rapid changes
      setTimeout(() => {
        if (ledOn && wsOpen()) {
          // Always use at least 35% brightness (0.35) unless user set it higher
          const effectiveBrightness = Math.max(0.35, brightness / 100);
          const commandData: Record<string, unknown> = {
            color: color1,
            mode: mode,
            pattern: pattern,
            interval: pattern !== 'static' ? intervalMs : 0,
            brightness: effectiveBrightness,
          };
          // Add color2 only when mode is dual
          if (mode === 'dual') {
            commandData.color2 = color2;
          }
          try {
            send(commandData);
            console.log('[LedModal] 🔄 Auto-applied settings:', JSON.stringify(commandData));
          } catch (error) {
            console.error('[LedModal] ❌ Auto-apply failed:', error);
          }
        }
      }, 300); // 300ms debounce
    }
  };

  // Handlers
  const handleColor1Change = (color: ColorResult) => {
    setColor1(color.hex);
    autoApplyIfOn();
  };
  const handleColor2Change = (color: ColorResult) => {
    setColor2(color.hex);
    autoApplyIfOn();
  };
  // Get available patterns for current mode
  const getAvailablePatterns = (): readonly LightingPattern[] => {
    switch (mode) {
      case 'single':
        return SINGLE_MODE_PATTERNS;
      case 'dual':
        return DUAL_MODE_PATTERNS;
      case 'rainbow':
        return RAINBOW_MODE_PATTERNS;
      default:
        return SINGLE_MODE_PATTERNS;
    }
  };

  const handleModeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newMode = e.target.value as LightingMode;
    setMode(newMode);
    
    // Reset pattern to first available pattern for new mode
    const availablePatterns = newMode === 'single' ? SINGLE_MODE_PATTERNS 
                           : newMode === 'dual' ? DUAL_MODE_PATTERNS 
                           : RAINBOW_MODE_PATTERNS;
    if (!availablePatterns.includes(pattern as any)) {
      setPattern(availablePatterns[0] as LightingPattern);
    }
    
    autoApplyIfOn();
  };
  const handlePatternChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const nextPattern = e.target.value as LightingPattern;
    setPattern(nextPattern);

    if (nextPattern === 'music' && intervalMs > 250) {
      setIntervalMs(120);
    }
    autoApplyIfOn();
  };
  const handleIntervalChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    setIntervalMs(Number.isFinite(value) ? Math.max(100, Math.floor(value)) : 100);
    autoApplyIfOn();
  };
  const handleBrightnessChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = Number(e.target.value);
    if (Number.isFinite(value)) setBrightness(Math.min(100, Math.max(0, Math.floor(value))));
    autoApplyIfOn();
  };

  const send = (payload: Record<string, unknown>) => {
    if (!wsOpen()) {
      console.error('[LedModal] ❌ Cannot send: WebSocket not open');
      console.error('[LedModal] WebSocket state:', ws.current?.readyState);
      throw new Error('WebSocket not open');
    }
    
    const jsonPayload = JSON.stringify(payload);
    console.log('[LedModal] 📤 Sending message:', jsonPayload);
    console.log('[LedModal] Message length:', jsonPayload.length, 'bytes');
    
    try {
      ws.current!.send(jsonPayload);
      console.log('[LedModal] ✅ Message sent successfully');
    } catch (error) {
      console.error('[LedModal] ❌ Failed to send message:', error);
      throw error;
    }
  };

  const handleTogglePower = (checked: boolean) => {
    const newState = checked;
    console.log(`[LedModal] 🔄 Toggle power: ${ledOn} -> ${newState}`);
    console.log(`[LedModal] WebSocket readyState: ${ws.current?.readyState} (OPEN=${WebSocket.OPEN}, CONNECTING=${WebSocket.CONNECTING})`);
    console.log(`[LedModal] Server status: ${serverStatus}`);
    
    if (!wsOpen()) {
      console.error('[LedModal] ⚠️ Lighting WS not open! Cannot toggle.');
      console.error('[LedModal] WebSocket object:', ws.current);
      console.error('[LedModal] ReadyState:', ws.current?.readyState);
      setServerStatus('disconnected');
      // Don't update state if WebSocket is not open
      return;
    }

    if (!newState) {
      // Turn OFF: Send pattern='off' command
      const offCommand = {
        color: '#000000',
        mode: 'single',
        pattern: 'off',
        interval: 0,
        brightness: 0.0, // Explicitly send as float 0.0 (backend expects 0-1 range)
      };
      console.log('[LedModal] 🔴 Sending LED OFF command:', JSON.stringify(offCommand));
      try {
        send(offCommand);
        console.log('[LedModal] ✅ OFF command sent successfully');
        // Update UI state after successful send
        setLedOn(false);
      } catch (error) {
        console.error('[LedModal] ❌ Failed to send OFF command:', error);
        // Don't update state on error - keep current state
      }
    } else {
      // Turn ON: Send complete command with current settings
      // Always use at least 35% brightness (0.35) unless user set it higher
      const effectiveBrightness = Math.max(0.35, brightness / 100);
      const onCommand: Record<string, unknown> = {
        color: color1,
        mode: mode,
        pattern: pattern,
        interval: pattern !== 'static' ? intervalMs : 0,
        brightness: effectiveBrightness, // Convert 0-100 to 0-1, minimum 0.35
      };
      // Add color2 only when mode is dual
      if (mode === 'dual') {
        onCommand.color2 = color2;
      }
      console.log('[LedModal] 🟢 Sending LED ON command:', JSON.stringify(onCommand));
      try {
        send(onCommand);
        console.log('[LedModal] ✅ ON command sent successfully');
        // Update UI state after successful send
        setLedOn(true);
      } catch (error) {
        console.error('[LedModal] ❌ Failed to send ON command:', error);
        // Don't update state on error - keep current state
      }
    }
  };

  const handleApply = () => {
    if (!wsOpen()) {
      console.error('Lighting WS not open.');
      return;
    }
    
    // If LED is off, turn it on with current settings
    if (!ledOn) {
      setLedOn(true);
      const onCommand = {
        color: color1,
        mode: mode,
        pattern: pattern,
        interval: pattern !== 'static' ? intervalMs : 0,
        brightness: brightness / 100, // Convert 0-100 to 0-1
      };
      console.log('[LedModal] 🟢 Applying settings and turning ON:', JSON.stringify(onCommand));
      try {
        send(onCommand);
        console.log('[LedModal] ✅ Settings applied successfully');
      } catch (error) {
        console.error('[LedModal] ❌ Failed to apply settings:', error);
        setLedOn(false); // Revert on error
      }
      return;
    }
    
    // Validate interval for dynamic patterns
    if (pattern !== 'static' && intervalMs <= 0) {
      console.error('Interval must be greater than 0 for dynamic patterns.');
      return;
    }
    
    // Send complete command payload matching backend LightingCommand struct
    // Backend expects: { color, color2 (optional), mode, pattern, interval, brightness }
    // Always use at least 35% brightness (0.35) unless user set it higher
    const effectiveBrightness = Math.max(0.35, brightness / 100);
    const commandData: Record<string, unknown> = {
      color: color1, // Hex string like "#ff0000"
      mode: mode,
      pattern: pattern,
      interval: pattern !== 'static' ? intervalMs : 0,
      brightness: effectiveBrightness, // Convert 0-100% to 0-1.0, minimum 0.35
    };
    // Add color2 only when mode is dual
    if (mode === 'dual') {
      commandData.color2 = color2;
    }
    console.log('[LedModal] 📤 Applying LED settings:', JSON.stringify(commandData));
    try {
      send(commandData);
      console.log('[LedModal] ✅ Settings applied successfully');
    } catch (error) {
      console.error('[LedModal] ❌ Failed to apply settings:', error);
    }
  };

  // STATUS PILL COLORS - Matching Omega theme
  const statusColor =
    serverStatus === 'connected'
      ? 'bg-[#00FF88] text-black'
      : serverStatus === 'connecting'
      ? 'bg-[#FFAA00] text-black'
      : 'bg-[#FF0066] text-white';

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
      className="fixed inset-0 bg-black/80 backdrop-blur-md flex justify-center items-center z-[9999] p-4"
      style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0 }}
      role="dialog"
      aria-modal="true"
      aria-labelledby="led-config-title"
    >
      <div 
        className="bg-[#0A0A0A] rounded-lg p-8 w-full max-w-2xl relative text-[#E0E0E0] shadow-2xl border-2 border-[#C400FF]/70 backdrop-blur-xl animate-in zoom-in-95 duration-200"
        style={{ 
          position: 'relative', 
          zIndex: 10000,
          boxShadow: '0 0 40px rgba(196, 0, 255, 0.3), 0 8px 32px rgba(0, 0, 0, 0.6), inset 0 0 40px rgba(196, 0, 255, 0.1)',
          fontFamily: "'Rajdhani', 'Exo 2', sans-serif"
        }}
      >
        {/* Close Button (absolute) */}
        <button
          type="button"
          className="absolute top-3 right-3 text-white bg-[#FF0066] hover:bg-[#FF0048] p-2 rounded focus:outline-none focus:ring-2 focus:ring-[#FF0066]/50 z-[10001] transition-all shadow-lg hover:shadow-[#FF0066]/50"
          style={{ position: 'absolute', zIndex: 10001 }}
          onClick={onClose}
          aria-label="Close LED configuration"
        >
          Close
        </button>

        {/* Title + Connection Status */}
        <div className="flex items-center justify-between border-b-2 border-[#00FF88]/70 pb-2 mb-4 pr-20">
          <h2 id="led-config-title" className="text-xl font-bold text-[#00FF88]" style={{ fontFamily: "'Orbitron', sans-serif", textShadow: '0 0 10px rgba(0, 255, 136, 0.5)' }}>
            LED Configuration
          </h2>
          <div
            className={`flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-semibold shadow-lg ${statusColor}`}
            style={{
              boxShadow: serverStatus === 'connected' 
                ? '0 0 15px rgba(0, 255, 136, 0.5)' 
                : serverStatus === 'connecting'
                ? '0 0 15px rgba(255, 170, 0, 0.5)'
                : '0 0 15px rgba(255, 0, 102, 0.5)'
            }}
            title="Lighting server connection status"
            aria-live="polite"
          >
            <span className={`inline-block w-2 h-2 rounded-full ${serverStatus === 'connected' ? 'bg-black' : 'bg-white/90'}`} />
            <span>{statusLabel}</span>
          </div>
        </div>

        {/* LED Power Toggle */}
        <div className="mt-2 mb-4 flex justify-between items-center rounded-lg border border-[#C400FF]/30 bg-[#1A1A1A]/50 px-4 py-3">
          <div className="flex-1">
            <div className="flex items-center gap-2 text-[#00FF88] font-semibold text-lg" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
              <span>LED Power:</span>
              <span className={ledOn ? 'text-[#00FF88]' : 'text-[#B0B0B0]'}>{ledOn ? 'On' : 'Off'}</span>
            </div>
            {serverStatus !== 'connected' && (
              <div className="text-[10px] text-[#FF0066] mt-1">Server unavailable - toggle disabled</div>
            )}
          </div>
          <Switch 
            checked={ledOn} 
            onCheckedChange={handleTogglePower}
            disabled={serverStatus !== 'connected'}
            aria-label="Toggle LED power"
            className="scale-110"
          />
        </div>

        <fieldset
          disabled={serverStatus !== 'connected'}
          className={serverStatus === 'connected' ? '' : 'opacity-50'}
        >
          {/* Color Picker 1 */}
          <label className="block text-[#00FF88] font-semibold mb-2 text-lg" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
            {mode === 'dual' ? 'Primary Color:' : 'Color:'}
          </label>
          <div className="border border-[#C400FF]/30 rounded-lg p-2 bg-[#1A1A1A]/50">
            <SketchPicker color={color1} onChange={handleColor1Change} />
          </div>

          {/* Color Picker 2 - Only show when mode is dual */}
          {mode === 'dual' && (
            <>
              <label className="block text-[#00FF88] font-semibold mb-2 text-lg mt-4" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
                Secondary Color:
              </label>
              <div className="border border-[#C400FF]/30 rounded-lg p-2 bg-[#1A1A1A]/50">
                <SketchPicker color={color2} onChange={handleColor2Change} />
              </div>
            </>
          )}

          {/* Mode Selector */}
          <div className="mt-4">
            <label htmlFor="mode" className="block text-[#00FF88] font-semibold mb-1" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
              Mode:
            </label>
            <select
              id="mode"
              value={mode}
              onChange={handleModeChange}
              className="w-full bg-[#1A1A1A] text-[#E0E0E0] p-2.5 rounded-lg border border-[#C400FF]/30 focus:border-[#C400FF] focus:ring-2 focus:ring-[#C400FF]/50 transition-all"
              style={{ 
                boxShadow: 'inset 0 2px 4px rgba(0, 0, 0, 0.3), 0 0 10px rgba(196, 0, 255, 0.1)'
              }}
            >
              {LIGHTING_MODES.map((m) => (
                <option key={m} value={m} className="bg-[#1A1A1A]">
                  {m.charAt(0).toUpperCase() + m.slice(1)}
                </option>
              ))}
            </select>
          </div>

          {/* Pattern Selector */}
          <div className="mt-4">
            <label htmlFor="pattern" className="block text-[#00FF88] font-semibold mb-1" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
              Pattern:
            </label>
            <select
              id="pattern"
              value={pattern}
              onChange={handlePatternChange}
              className="w-full bg-[#1A1A1A] text-[#E0E0E0] p-2.5 rounded-lg border border-[#C400FF]/30 focus:border-[#C400FF] focus:ring-2 focus:ring-[#C400FF]/50 transition-all"
              style={{ 
                boxShadow: 'inset 0 2px 4px rgba(0, 0, 0, 0.3), 0 0 10px rgba(196, 0, 255, 0.1)'
              }}
            >
              {LIGHTING_PATTERNS.map((p) => (
                <option key={p} value={p} className="bg-[#1A1A1A]">
                  {p.charAt(0).toUpperCase() + p.slice(1)}
                </option>
              ))}
            </select>
            {pattern === 'music' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                Music mode listens to the default microphone when available and generates a
                beat-reactive light show. Reduce the interval for faster reaction times.
              </p>
            )}
            {pattern === 'fade' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                Fade creates smooth color transitions. Adjust interval to control fade speed.
              </p>
            )}
            {pattern === 'chase' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                Chase creates a moving light effect across the LED strip. Lower interval = faster chase.
              </p>
            )}
            {pattern === 'lightshow' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                Lightshow creates a multi-stage animated display with rotating color bands and sparkles.
              </p>
            )}
            {pattern === 'rainbow' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                Rainbow displays a full spectrum sweep across all LEDs. Interval controls animation speed.
              </p>
            )}
            {pattern === 'rave' && (
              <p className="mt-2 text-sm text-[#C400FF] font-bold bg-[#1A1A1A]/70 p-2 rounded border border-[#C400FF]/50" style={{ textShadow: '0 0 8px rgba(196, 0, 255, 0.6)' }}>
                🎉 RAVE MODE: Energetic dancing lights with fast color cycling, strobing, and wave effects!
                No audio required - creates synthetic beat patterns. Lower interval = faster party! 🎊
              </p>
            )}
            {pattern === 'breathing' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                💨 BREATHING: Smooth, calming pulse effect perfect for idle/standby mode. Energy-efficient.
              </p>
            )}
            {pattern === 'aurora' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                🌌 AURORA: Flowing northern lights effect with organic wave movements. Mesmerizing ambient lighting.
              </p>
            )}
            {pattern === 'matrix' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                💚 MATRIX: Cool Matrix-style rain effect with falling light trails. Perfect for tech aesthetic!
              </p>
            )}
            {pattern === 'fire' && (
              <p className="mt-2 text-sm text-[#00FF88]/90 bg-[#1A1A1A]/50 p-2 rounded border border-[#C400FF]/20" style={{ textShadow: '0 0 4px rgba(0, 255, 136, 0.3)' }}>
                🔥 FIRE: Realistic flickering fire effect with orange/red/yellow flames. Great for ambient lighting!
              </p>
            )}
          </div>

          {/* Interval Input */}
          {pattern !== 'static' && (
            <div className="mt-4">
              <label htmlFor="interval" className="block text-[#00FF88] font-semibold mb-1" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
                Interval (ms):
              </label>
              <input
                id="interval"
                type="number"
                value={intervalMs}
                onChange={handleIntervalChange}
                className="w-full bg-[#1A1A1A] text-[#E0E0E0] p-2.5 rounded-lg border border-[#C400FF]/30 focus:border-[#C400FF] focus:ring-2 focus:ring-[#C400FF]/50 transition-all"
                style={{ 
                  boxShadow: 'inset 0 2px 4px rgba(0, 0, 0, 0.3), 0 0 10px rgba(196, 0, 255, 0.1)'
                }}
                min={intervalConfig.min}
                step={intervalConfig.step}
              />
            </div>
          )}

          {/* Brightness Slider */}
          <div className="mt-4">
            <label htmlFor="brightness" className="block text-[#00FF88] font-semibold mb-2" style={{ textShadow: '0 0 8px rgba(0, 255, 136, 0.4)' }}>
              Brightness (%): <span className="text-[#C400FF]">{brightness}%</span>
            </label>
            <input
              id="brightness"
              type="range"
              min={0}
              max={100}
              value={brightness}
              onChange={handleBrightnessChange}
              className="w-full h-2 bg-[#1A1A1A] rounded-lg appearance-none cursor-pointer accent-[#C400FF]"
              style={{
                background: `linear-gradient(to right, #C400FF 0%, #C400FF ${brightness}%, #1A1A1A ${brightness}%, #1A1A1A 100%)`,
                boxShadow: '0 0 10px rgba(196, 0, 255, 0.3)'
              }}
            />
          </div>
        </fieldset>

        {/* Apply Button */}
        <button
          onClick={handleApply}
          disabled={serverStatus !== 'connected'}
          className={`text-white p-3.5 rounded-lg mt-6 w-full focus:outline-none focus:ring-2 transition-all font-semibold ${
            serverStatus !== 'connected'
              ? 'bg-[#2A2A2A] cursor-not-allowed border border-[#4A4A4A]'
              : ledOn
              ? 'bg-gradient-to-r from-[#C400FF] to-[#8B00FF] hover:from-[#D400FF] hover:to-[#9B00FF] focus:ring-[#C400FF]/50 border border-[#C400FF]/50 shadow-lg hover:shadow-[#C400FF]/50'
              : 'bg-gradient-to-r from-[#00FF88] to-[#00DD77] hover:from-[#00FF99] hover:to-[#00EE88] focus:ring-[#00FF88]/50 border border-[#00FF88]/50 shadow-lg hover:shadow-[#00FF88]/50 text-black'
          }`}
          style={{
            textShadow: serverStatus === 'connected' ? '0 0 8px rgba(196, 0, 255, 0.6)' : 'none',
            boxShadow: serverStatus === 'connected' ? '0 0 20px rgba(196, 0, 255, 0.4)' : 'none'
          }}
        >
          {serverStatus !== 'connected' 
            ? 'Server Unavailable' 
            : ledOn 
            ? 'Apply Settings' 
            : 'Turn On & Apply Settings'}
        </button>
      </div>
    </div>,
    document.body
  );
};

export default LedModal;
