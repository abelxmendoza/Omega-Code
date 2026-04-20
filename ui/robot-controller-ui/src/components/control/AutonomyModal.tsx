/*
  AutonomyModal — simplified to the three modes that actually run hardware on Omega-1:
    • Avoid Obstacles  (ObstacleAvoidanceMode — ultrasonic loop, real motors)
    • ArUco Seek       (ArucoSeekMode — camera + motor seek loop)
    • Dock             (DockMode — sends dock command, logs confirmation)

  Parameter translation: UI values are converted to the backend's expected names and
  units inside toBackendParams() before onStart is called, so the backend actually
  receives correct values (previous version sent wrong names and wrong units).
*/

'use client';

import React, { useState } from 'react';
import Link from 'next/link';
import {
  Bot, Play, Square, Shield, AlertTriangle, CheckCircle,
  ChevronDown, ChevronUp, Save, Upload, Info,
} from 'lucide-react';
import { useServiceSafety } from '@/hooks/useServiceSafety';

import { Button }                                   from '@/components/ui/button';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from '@/components/ui/dialog';
import { Switch }                                   from '@/components/ui/switch';
import { Slider }                                   from '@/components/ui/slider';
import { Input }                                    from '@/components/ui/input';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

export type AutonomyMode = 'idle' | 'avoid_obstacles' | 'aruco' | 'dock'
  // kept for backward-compat with index.tsx demo-mode switch
  | 'line_follow' | 'patrol' | 'waypoints' | 'aruco_seek';

/** UI-facing parameters — clean names, sensible units. */
export type AutonomyParams = {
  speedPct:       number;   // move speed 0–100 %
  stopDistCm:     number;   // obstacle avoidance — hard-stop distance (cm)
  warnDistCm:     number;   // obstacle avoidance — start slowing (cm)
  cameraAssist:   boolean;  // obstacle avoidance — use camera as 2nd sensor
  detectOnly:     boolean;  // obstacle avoidance — react only, don't drive
  arucoStopDistM: number;   // aruco seek — stop distance from marker (m)
  arucoTargetId:  number;   // aruco seek — -1 = any, 0+ = specific marker ID
  arucoMarkerM:   number;   // aruco seek — physical marker side length (m)
  batteryMinPct:  number;   // safety — stop below this battery level
  headlights:     boolean;  // auto-lights permission (passed to lighting API)
};

export type AutonomyModalProps = {
  className?:    string;
  initialMode?:  AutonomyMode;
  connected?:    boolean;
  autonomyActive?: boolean;
  batteryPct?:   number;
  triggerLabel?: string;
  /** Live FSM state string — shown as a badge when autonomy is active. */
  fsmState?:     string;
  onStart?:  (mode: AutonomyMode, params: Record<string, unknown>) => Promise<void> | void;
  onStop?:   () => Promise<void> | void;
  onDock?:   () => Promise<void> | void;
  onUpdate?: (params: Record<string, unknown>) => Promise<void> | void;
  /** @deprecated GPS waypoints are not used on Omega-1; kept for API compat */
  onSetWaypoint?: (label: string, lat: number, lon: number) => Promise<void> | void;
};

// ---------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------

const DEFAULTS: AutonomyParams = {
  speedPct:       40,
  stopDistCm:     25,
  warnDistCm:     50,
  cameraAssist:   false,
  detectOnly:     false,
  arucoStopDistM: 0.40,
  arucoTargetId:  -1,
  arucoMarkerM:   0.10,
  batteryMinPct:  15,
  headlights:     false,
};

// ---------------------------------------------------------------------------
// Parameter translation — UI units → backend names and units
// ---------------------------------------------------------------------------

function toBackendParams(mode: AutonomyMode, p: AutonomyParams): Record<string, unknown> {
  const fwd = Math.max(0.1, Math.min(1.0, p.speedPct / 100));

  switch (mode) {
    case 'avoid_obstacles':
      return {
        forward_speed:  fwd,
        stop_cm:        p.stopDistCm,
        warn_cm:        p.warnDistCm,
        drive:          !p.detectOnly,
        camera_assist:  p.cameraAssist,
      };

    case 'aruco':
    case 'aruco_seek':
      return {
        forward_speed:  fwd * 0.7,   // approach speed (gentler than top speed)
        search_speed:   fwd * 0.5,   // rotation speed while searching
        stop_dist:      p.arucoStopDistM,
        target_id:      p.arucoTargetId,
        marker_length:  p.arucoMarkerM,
      };

    case 'dock':
      return { forward_speed: fwd };

    default:
      return { forward_speed: fwd };
  }
}

// ---------------------------------------------------------------------------
// Mode definitions
// ---------------------------------------------------------------------------

const MODES = [
  {
    value:   'avoid_obstacles' as AutonomyMode,
    label:   'Avoid Obstacles',
    icon:    '🛡',
    summary: 'Robot drives forward and automatically steers around anything in its path using the ultrasonic sensor.',
    needs:   'Ultrasonic sensor',
  },
  {
    value:   'aruco' as AutonomyMode,
    label:   'ArUco Seek',
    icon:    '◎',
    summary: 'Robot searches by rotating, locks onto an ArUco marker with the camera, aligns itself, and stops in front of it.',
    needs:   'Camera + ArUco marker',
  },
  {
    value:   'dock' as AutonomyMode,
    label:   'Return to Dock',
    icon:    '⌂',
    summary: 'Sends a dock command — robot will attempt to return to its home/charging position.',
    needs:   'Dock position configured',
  },
] as const;

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

export default function AutonomyModal({
  className,
  initialMode = 'avoid_obstacles',
  connected   = true,
  autonomyActive = false,
  batteryPct  = 100,
  triggerLabel = 'Autonomy',
  fsmState,
  onStart,
  onStop,
  onDock,
}: AutonomyModalProps) {
  const [open, setOpen]     = useState(false);
  const [mode, setMode]     = useState<AutonomyMode>(
    initialMode === 'idle' ? 'avoid_obstacles' : initialMode,
  );
  const [params, setParams] = useState<AutonomyParams>({ ...DEFAULTS });
  const [busy,  setBusy]    = useState(false);
  const [showAdv, setShowAdv] = useState(false);
  const fileRef = React.useRef<HTMLInputElement>(null);

  const { movementReady, sensorsReady, blockedReason } = useServiceSafety();
  const safetyBlocked = !movementReady || !sensorsReady;

  function setParam<K extends keyof AutonomyParams>(key: K, value: AutonomyParams[K]) {
    setParams(prev => ({ ...prev, [key]: value }));
  }

  async function handleStart() {
    if (safetyBlocked) return;
    setBusy(true);
    try {
      await onStart?.(mode, toBackendParams(mode, params));
    } catch (e) {
      console.warn('autonomy start failed:', e);
    } finally {
      setBusy(false);
    }
  }

  async function handleStop() {
    setBusy(true);
    try { await onStop?.(); }
    catch (e) { console.warn('autonomy stop failed:', e); }
    finally { setBusy(false); }
  }

  async function handleDock() {
    setBusy(true);
    try { await onDock?.(); }
    catch (e) { console.warn('autonomy dock failed:', e); }
    finally { setBusy(false); }
  }

  function exportConfig() {
    const blob = new Blob([JSON.stringify({ mode, params }, null, 2)], { type: 'application/json' });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    a.href     = url;
    a.download = `omega-autonomy-${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
  }

  function onFileSelected(e: React.ChangeEvent<HTMLInputElement>) {
    const f = e.target.files?.[0];
    if (!f) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const json = JSON.parse(String(reader.result || '{}'));
        if (json.mode)   setMode(json.mode);
        if (json.params) setParams(p => ({ ...p, ...json.params }));
      } catch { /* ignore malformed file */ }
    };
    reader.readAsText(f);
    e.target.value = '';
  }

  const selectedMode = MODES.find(m => m.value === mode);
  const statusColor  = !connected       ? 'text-red-400'
                     : autonomyActive   ? 'text-emerald-400'
                     :                   'text-amber-400';
  const statusLabel  = !connected       ? 'Offline'
                     : autonomyActive   ? 'Running'
                     :                   'Ready';

  return (
    <div className={className}>
      <Dialog open={open} onOpenChange={setOpen}>
        <DialogTrigger asChild>
          <Button
            className="mt-3 gap-2 border-0 bg-gradient-to-r from-emerald-600 via-teal-600 to-cyan-600
              hover:from-emerald-500 hover:via-teal-500 hover:to-cyan-500
              text-white shadow-lg shadow-emerald-500/25"
            aria-label="Open Autonomy settings"
          >
            <Bot className="h-4 w-4" /> {triggerLabel}
          </Button>
        </DialogTrigger>

        <DialogContent className="sm:max-w-md max-h-[88vh] p-0 overflow-hidden border border-neutral-800 bg-neutral-950 text-neutral-100 flex flex-col">

          {/* ── Header ─────────────────────────────────────── */}
          <DialogHeader className="px-5 pt-5 pb-3 flex-shrink-0 border-b border-neutral-800">
            <div className="flex items-center justify-between">
              <DialogTitle className="flex items-center gap-2 text-base">
                <Bot className="h-5 w-5 text-emerald-400" />
                Autonomous Mode
              </DialogTitle>
              <div className="flex items-center gap-2">
                {autonomyActive && fsmState && (
                  <span className="text-[10px] font-mono font-bold px-1.5 py-0.5 rounded bg-emerald-900/50 border border-emerald-600/40 text-emerald-300 tracking-widest">
                    {fsmState}
                  </span>
                )}
                <span className={`text-xs font-semibold ${statusColor}`}>
                  ● {statusLabel}
                </span>
              </div>
            </div>
            <p className="text-xs text-neutral-400 mt-1 leading-snug">
              Choose a behavior, adjust settings for your environment, then press <strong>Start</strong>.
              Hit <strong>Stop</strong> any time to halt the robot immediately.
            </p>
          </DialogHeader>

          <div className="px-5 py-4 flex flex-col gap-4 overflow-y-auto flex-1 min-h-0">

            {/* ── Safety gate ────────────────────────────────── */}
            {safetyBlocked && (
              <div className="flex items-start gap-2 px-3 py-2.5 rounded-md bg-rose-600/15 border border-rose-500/40" role="alert">
                <AlertTriangle className="h-4 w-4 text-rose-400 shrink-0 mt-0.5" />
                <div>
                  <p className="text-xs font-bold text-rose-300">Cannot start — services offline</p>
                  <p className="text-[11px] text-rose-400 mt-0.5">{blockedReason}</p>
                  <p className="text-[10px] text-rose-500 mt-1">
                    Go to{' '}
                    <Link href="/services" className="underline hover:text-rose-300">
                      Service Management
                    </Link>{' '}
                    to start the required services.
                  </p>
                </div>
              </div>
            )}

            {/* ── Mode picker ─────────────────────────────────── */}
            <div>
              <label className="text-xs font-semibold text-neutral-300 uppercase tracking-wide block mb-2">
                Behavior
              </label>
              <div className="grid grid-cols-1 gap-2">
                {MODES.map(m => (
                  <button
                    key={m.value}
                    type="button"
                    onClick={() => setMode(m.value)}
                    className={[
                      'text-left rounded-lg border px-3 py-2.5 transition-colors',
                      mode === m.value
                        ? 'bg-emerald-900/40 border-emerald-500/60'
                        : 'bg-neutral-900 border-neutral-800 hover:border-neutral-700',
                    ].join(' ')}
                  >
                    <div className="flex items-center gap-2">
                      <span className="text-base leading-none">{m.icon}</span>
                      <div>
                        <div className="text-sm font-medium text-neutral-100">{m.label}</div>
                        <div className="text-[11px] text-neutral-400 mt-0.5 leading-snug">{m.summary}</div>
                        <div className="text-[10px] text-neutral-600 mt-0.5">Requires: {m.needs}</div>
                      </div>
                      {mode === m.value && (
                        <CheckCircle className="h-4 w-4 text-emerald-400 ml-auto shrink-0" />
                      )}
                    </div>
                  </button>
                ))}
              </div>
            </div>

            {/* ── Speed ───────────────────────────────────────── */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="text-xs font-semibold text-neutral-300 uppercase tracking-wide">
                  Move Speed
                </label>
                <span className="text-xs font-bold text-neutral-100 tabular-nums">{params.speedPct}%</span>
              </div>
              <Slider
                value={[params.speedPct]}
                onValueChange={([v]) => setParam('speedPct', v ?? 40)}
                min={10} max={100} step={5}
                aria-label="Move speed"
              />
              <div className="flex justify-between text-[10px] text-neutral-500 mt-1">
                <span>Slow (safer)</span>
                <span>Start here: 30–40%</span>
                <span>Fast</span>
              </div>
            </div>

            {/* ── Mode-specific settings ──────────────────────── */}
            {mode === 'avoid_obstacles' && (
              <div className="rounded-lg border border-neutral-800 bg-neutral-900/60 p-3 space-y-3">
                <div className="text-xs font-semibold text-neutral-300 uppercase tracking-wide">
                  Obstacle Avoidance Settings
                </div>

                <div className="grid grid-cols-2 gap-3">
                  <div>
                    <label className="text-[11px] text-neutral-400 block mb-1">
                      Hard Stop Distance
                    </label>
                    <div className="flex items-center gap-1.5">
                      <Input
                        type="number"
                        value={params.stopDistCm}
                        min={5} max={80} step={1}
                        className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs h-8"
                        onChange={e => {
                          const n = parseInt(e.target.value, 10);
                          if (isFinite(n)) setParam('stopDistCm', Math.max(5, Math.min(80, n)));
                        }}
                      />
                      <span className="text-[11px] text-neutral-500 shrink-0">cm</span>
                    </div>
                    <p className="text-[10px] text-neutral-500 mt-0.5">
                      Stop &amp; pivot when an obstacle is this close. Default: 25 cm.
                    </p>
                  </div>

                  <div>
                    <label className="text-[11px] text-neutral-400 block mb-1">
                      Slow-Down Distance
                    </label>
                    <div className="flex items-center gap-1.5">
                      <Input
                        type="number"
                        value={params.warnDistCm}
                        min={20} max={200} step={5}
                        className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs h-8"
                        onChange={e => {
                          const n = parseInt(e.target.value, 10);
                          if (isFinite(n)) setParam('warnDistCm', Math.max(20, Math.min(200, n)));
                        }}
                      />
                      <span className="text-[11px] text-neutral-500 shrink-0">cm</span>
                    </div>
                    <p className="text-[10px] text-neutral-500 mt-0.5">
                      Begin braking when an obstacle is this close. Default: 50 cm.
                    </p>
                  </div>
                </div>

                <ToggleRow
                  label="Detect-only (no driving)"
                  description="Robot stays still but still stops if something enters the stop zone. Good for testing sensor readings."
                  checked={params.detectOnly}
                  onCheckedChange={v => setParam('detectOnly', v)}
                />
                <ToggleRow
                  label="Camera assist"
                  description="Use the camera as a second obstacle sensor in addition to the ultrasonic sensor. Requires CSI camera connected."
                  checked={params.cameraAssist}
                  onCheckedChange={v => setParam('cameraAssist', v)}
                />
              </div>
            )}

            {mode === 'aruco' && (
              <div className="rounded-lg border border-neutral-800 bg-neutral-900/60 p-3 space-y-3">
                <div className="text-xs font-semibold text-neutral-300 uppercase tracking-wide">
                  ArUco Seek Settings
                </div>

                <div className="grid grid-cols-2 gap-3">
                  <div>
                    <label className="text-[11px] text-neutral-400 block mb-1">
                      Stop Distance
                    </label>
                    <div className="flex items-center gap-1.5">
                      <Input
                        type="number"
                        value={params.arucoStopDistM}
                        min={0.1} max={2.0} step={0.05}
                        className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs h-8"
                        onChange={e => {
                          const n = parseFloat(e.target.value);
                          if (isFinite(n)) setParam('arucoStopDistM', Math.max(0.1, Math.min(2.0, n)));
                        }}
                      />
                      <span className="text-[11px] text-neutral-500 shrink-0">m</span>
                    </div>
                    <p className="text-[10px] text-neutral-500 mt-0.5">
                      Stop this far in front of the marker. Default: 0.40 m.
                    </p>
                  </div>

                  <div>
                    <label className="text-[11px] text-neutral-400 block mb-1">
                      Target Marker ID
                    </label>
                    <Input
                      type="number"
                      value={params.arucoTargetId}
                      min={-1} max={999} step={1}
                      className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs h-8"
                      onChange={e => {
                        const n = parseInt(e.target.value, 10);
                        if (isFinite(n)) setParam('arucoTargetId', Math.max(-1, n));
                      }}
                    />
                    <p className="text-[10px] text-neutral-500 mt-0.5">
                      -1 = lock onto the nearest marker. 0, 1, 2… = seek that specific ID.
                    </p>
                  </div>
                </div>

                <div>
                  <label className="text-[11px] text-neutral-400 block mb-1">
                    Marker Physical Size
                  </label>
                  <div className="flex items-center gap-1.5">
                    <Input
                      type="number"
                      value={params.arucoMarkerM}
                      min={0.03} max={0.5} step={0.01}
                      className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs h-8 w-28"
                      onChange={e => {
                        const n = parseFloat(e.target.value);
                        if (isFinite(n)) setParam('arucoMarkerM', Math.max(0.03, Math.min(0.5, n)));
                      }}
                    />
                    <span className="text-[11px] text-neutral-500 shrink-0">m</span>
                  </div>
                  <p className="text-[10px] text-neutral-500 mt-0.5">
                    The printed side length of your ArUco marker. Accurate size = accurate distance estimate.
                    Default: 0.10 m (10 cm).
                  </p>
                </div>

                <div className="flex items-start gap-2 px-2.5 py-2 rounded bg-indigo-900/20 border border-indigo-800/40 text-[11px] text-indigo-300">
                  <Info className="h-3.5 w-3.5 mt-0.5 shrink-0" />
                  <span>
                    The robot will rotate in place until it sees a marker, then align and approach.
                    Make sure the camera is pointing forward and the marker is within line of sight.
                  </span>
                </div>
              </div>
            )}

            {mode === 'dock' && (
              <div className="rounded-lg border border-neutral-800 bg-neutral-900/60 p-3">
                <div className="flex items-start gap-2 text-[11px] text-neutral-400 leading-snug">
                  <Info className="h-3.5 w-3.5 mt-0.5 shrink-0 text-amber-400" />
                  <span>
                    Sends the dock command to the backend. The robot will attempt to return to its home
                    position. Make sure the path back is clear before starting.
                    You can also use the <strong className="text-neutral-300">Return to Dock</strong> button below as a shortcut.
                  </span>
                </div>
              </div>
            )}

            {/* ── Safety check ─────────────────────────────────── */}
            {batteryPct < params.batteryMinPct && (
              <div className="flex items-center gap-2 px-3 py-2 rounded bg-red-900/20 border border-red-700/40 text-[11px] text-red-300">
                <AlertTriangle className="h-3.5 w-3.5 shrink-0" />
                Battery {batteryPct}% is below your safety floor ({params.batteryMinPct}%).
                Charge the robot before running autonomy.
              </div>
            )}
            {batteryPct >= params.batteryMinPct && connected && (
              <div className="flex items-center gap-2 px-3 py-2 rounded bg-emerald-900/20 border border-emerald-700/40 text-[11px] text-emerald-300">
                <CheckCircle className="h-3.5 w-3.5 shrink-0" />
                Ready — press Start to begin.
              </div>
            )}

            {/* ── Start / Stop / Dock ──────────────────────────── */}
            <div className="grid grid-cols-2 gap-2">
              <Button
                disabled={busy || !connected || safetyBlocked || batteryPct < params.batteryMinPct}
                onClick={handleStart}
                className="gap-2"
                title={safetyBlocked ? blockedReason ?? 'Services offline' : undefined}
              >
                <Play className="h-4 w-4" />
                {busy ? 'Starting…' : 'Start'}
              </Button>
              <Button
                disabled={busy}
                variant="destructive"
                onClick={handleStop}
                className="gap-2"
              >
                <Square className="h-4 w-4" /> Stop
              </Button>
            </div>

            <Button
              disabled={busy}
              variant="secondary"
              onClick={handleDock}
              className="w-full gap-2"
            >
              <Bot className="h-4 w-4" /> Return to Dock
            </Button>

            {/* ── Advanced ────────────────────────────────────── */}
            <button
              type="button"
              onClick={() => setShowAdv(v => !v)}
              className="flex items-center justify-between w-full text-xs text-neutral-400 hover:text-neutral-200 transition-colors py-1"
            >
              <div className="flex items-center gap-1.5">
                <Shield className="h-3.5 w-3.5" />
                Advanced &amp; Safety
              </div>
              {showAdv ? <ChevronUp className="h-3.5 w-3.5" /> : <ChevronDown className="h-3.5 w-3.5" />}
            </button>

            {showAdv && (
              <div className="rounded-lg border border-neutral-800 bg-neutral-900/60 p-3 space-y-3">
                <div>
                  <label className="text-[11px] text-neutral-400 block mb-1">
                    Battery Safety Floor
                  </label>
                  <div className="flex items-center gap-1.5">
                    <Input
                      type="number"
                      value={params.batteryMinPct}
                      min={0} max={50} step={1}
                      className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs h-8 w-20"
                      onChange={e => {
                        const n = parseInt(e.target.value, 10);
                        if (isFinite(n)) setParam('batteryMinPct', Math.max(0, Math.min(50, n)));
                      }}
                    />
                    <span className="text-[11px] text-neutral-500">%</span>
                  </div>
                  <p className="text-[10px] text-neutral-500 mt-0.5">
                    Block start if battery is below this level. Default: 15%.
                  </p>
                </div>

                <div className="border-t border-neutral-800/50 pt-3">
                  <div className="text-[11px] text-neutral-400 mb-2">Save / Load Config</div>
                  <div className="flex gap-2">
                    <Button variant="secondary" className="gap-1.5 text-xs h-8" onClick={exportConfig}>
                      <Save className="h-3.5 w-3.5" /> Export JSON
                    </Button>
                    <Button variant="secondary" className="gap-1.5 text-xs h-8" onClick={() => fileRef.current?.click()}>
                      <Upload className="h-3.5 w-3.5" /> Import JSON
                    </Button>
                    <input ref={fileRef} type="file" accept="application/json" className="hidden" onChange={onFileSelected} />
                  </div>
                </div>
              </div>
            )}

          </div>

          {/* ── Footer ──────────────────────────────────────── */}
          <div className="px-5 py-3 border-t border-neutral-800 flex-shrink-0">
            <Button variant="secondary" className="w-full" onClick={() => setOpen(false)}>
              Close
            </Button>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  );
}

// ---------------------------------------------------------------------------
// ToggleRow helper
// ---------------------------------------------------------------------------

function ToggleRow({
  label, description, checked, onCheckedChange,
}: {
  label: string;
  description?: string;
  checked: boolean;
  onCheckedChange: (v: boolean) => void;
}) {
  return (
    <div className="flex items-center justify-between rounded-lg border border-neutral-800 bg-neutral-950 px-2.5 py-2 gap-3">
      <div className="flex-1 min-w-0">
        <div className="text-xs font-medium text-neutral-100">{label}</div>
        {description && (
          <div className="text-[10px] text-neutral-400 mt-0.5 leading-snug">{description}</div>
        )}
      </div>
      <Switch
        checked={checked}
        onCheckedChange={onCheckedChange}
        aria-label={label}
        className="scale-75 shrink-0"
      />
    </div>
  );
}
