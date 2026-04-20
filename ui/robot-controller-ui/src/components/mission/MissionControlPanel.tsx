/**
 * MissionControlPanel — robot status + waypoint mission control sidebar.
 *
 * Displays:
 *   - Live pose (x, y, θ) from the WebSocket pose stream
 *   - EKF quality indicator (colour-coded)
 *   - ArUco fix count + last marker seen
 *   - Waypoint list with progress indicator
 *   - Start / Pause / Abort mission buttons → POST /sim/* or /navigation/*
 *
 * Intentionally contains NO canvas drawing — that is MissionMap's job.
 * All props flow down from the /mission page which owns shared state.
 */

'use client';

import React, { useCallback, useState } from 'react';
import type { PoseData, PoseStreamStatus } from '@/hooks/usePoseStream';
import type { MissionState } from '@/hooks/useMissionStream';
import type { Waypoint } from './MissionMap';
import { buildGatewayUrl } from '@/config/gateway';
import SimLauncherCard from './SimLauncherCard';
import { useDemoMode } from '@/context/DemoModeContext';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

export interface MissionControlPanelProps {
  pose:                   PoseData | null;
  status:                 PoseStreamStatus;
  correctionCount:        number;
  lastMarkerSeen:         number | null;
  waypoints:              Waypoint[];
  activeWpIndex:          number;
  /** Authoritative mission state from useMissionStream */
  missionState:           MissionState;
  /** Whether the /ws/mission WebSocket is connected */
  missionStreamConnected: boolean;
  onClearWaypoints:       () => void;
  /** Called after a scenario is successfully loaded — parent updates map */
  onScenarioLoaded:       (name: string) => void;
  /**
   * Demo mode only — true when SimEngine is actively navigating in a mode that
   * doesn't emit a positive wpIdx (e.g. aruco_seek).
   */
  demoIsNavigating?:      boolean;
  /** Demo mode only — human-readable label for the active behavior */
  demoBehaviorLabel?:     string;
  /** Demo mode only — called after SimEngine.reset() to clear map trail + waypoints */
  onDemoReset?:           () => void;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function qualityClass(q: number): string {
  if (q >= 0.7)  return 'text-emerald-400';
  if (q >= 0.35) return 'text-amber-400';
  return 'text-red-400';
}

function qualityBgClass(q: number): string {
  if (q >= 0.7)  return 'bg-emerald-900/40 border-emerald-600/40';
  if (q >= 0.35) return 'bg-amber-900/40 border-amber-600/40';
  return 'bg-red-900/40 border-red-600/40';
}

function qualityLabel(q: number): string {
  if (q >= 0.7)  return 'Good';
  if (q >= 0.35) return 'Fair';
  if (q > 0)     return 'Poor';
  return 'Dead';
}

function statusDot(s: PoseStreamStatus): React.ReactNode {
  const color =
    s === 'connected'    ? 'bg-emerald-400' :
    s === 'connecting'   ? 'bg-amber-400 animate-pulse' :
    s === 'error'        ? 'bg-red-500' :
                           'bg-gray-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} />;
}

function headingArrow(deg: number): string {
  // 8-point compass
  const dirs = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘'];
  const idx   = Math.round(((deg % 360) + 360) % 360 / 45) % 8;
  return dirs[idx];
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

const MissionControlPanel: React.FC<MissionControlPanelProps> = ({
  pose,
  status,
  correctionCount,
  lastMarkerSeen,
  waypoints,
  activeWpIndex,
  missionState,
  missionStreamConnected,
  onClearWaypoints,
  onScenarioLoaded,
  demoIsNavigating  = false,
  demoBehaviorLabel = '',
  onDemoReset,
}) => {
  const { demoMode, setDemoMode, engine: simEngine } = useDemoMode();

  const [missionError,    setMissionError]    = useState<string | null>(null);
  const [simLoaded,       setSimLoaded]       = useState(false);
  const [starting,        setStarting]        = useState(false);
  const [missionLaunched, setMissionLaunched] = useState(false);

  // Sim backend is always started locally by sim-launcher — never use the
  // gateway URL here, which points at the Pi when on LAN/Tailscale profiles.
  const simBase = 'http://localhost:8000';
  // Live robot endpoints (EKF reset etc.) still use the gateway.
  // buildGatewayUrl('') returns a trailing slash — strip it to avoid double-slash.
  const apiBase = buildGatewayUrl('').replace(/\/$/, '');

  // ---------------------------------------------------------------------------
  // Actions
  // ---------------------------------------------------------------------------

  const startMission = useCallback(async () => {
    if (waypoints.length === 0) {
      setMissionError('Add at least one waypoint on the map first.');
      return;
    }
    setMissionError(null);
    setMissionLaunched(false);

    // ── Frontend-demo mode: drive SimEngine directly, no backend needed ──
    if (demoMode) {
      simEngine.navigateTo(waypoints.map((w) => ({ x: w.x, y: w.y })));
      return;
    }

    // ── Sim-backend / live mode: call the backend API ────────────────────
    setStarting(true);
    try {
      // Step 1 — Start the sim loop only if it is not already running.
      const statusRes = await fetch(`${simBase}/sim/status`);
      const simStatus = statusRes.ok ? await statusRes.json() : null;
      if (!simStatus?.running) {
        const startRes = await fetch(`${simBase}/sim/start`, {
          method:  'POST',
          headers: { 'Content-Type': 'application/json' },
          body:    JSON.stringify({ x: 0, y: 0, theta: 0, hz: 20, reset_ekf: true }),
        });
        if (!startRes.ok) {
          if (startRes.status === 404) {
            throw new Error('Sim routes not found — start the backend with SIM_MODE=1');
          }
          throw new Error(`Sim start failed: ${startRes.status}`);
        }
      }

      // Step 2 — Post the waypoint mission
      const res = await fetch(`${simBase}/sim/mission`, {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({
          waypoints:      waypoints.map((w) => ({ x: w.x, y: w.y, label: w.label })),
          goal_tolerance: 0.15,
          max_v:          0.30,
          max_omega:      0.80,
        }),
      });
      if (!res.ok) throw new Error(`Mission load failed: ${res.status}`);

      // Mark as launched — missionState will flip to 'running' when the
      // mission_started event arrives on /ws/mission.
      setMissionLaunched(true);
    } catch (err) {
      setMissionError(err instanceof Error ? err.message : String(err));
    } finally {
      setStarting(false);
    }
  }, [demoMode, simEngine, waypoints]);

  const abortMission = useCallback(async () => {
    setMissionLaunched(false);
    if (demoMode) {
      simEngine.cancelNav();
      return;
    }
    try {
      await fetch(`${simBase}/sim/mission/abort`, { method: 'POST' });
    } catch { /* best-effort */ }
  }, [demoMode, simEngine]);

  const loadScenario = useCallback((name: string) => {
    setMissionError(null);
    // Scenario data is baked into the frontend — always update the map immediately.
    // Also auto-switch to frontend-demo so Start Mission uses SimEngine rather than
    // trying to reach localhost:8000 (which is blocked on Vercel and other deployments).
    if (!demoMode) setDemoMode(true);
    setSimLoaded(true);
    onScenarioLoaded(name);
  }, [demoMode, setDemoMode, onScenarioLoaded]);

  const resetEkf = useCallback(async () => {
    if (demoMode) {
      simEngine.reset();
      onDemoReset?.();
      return;
    }
    try {
      await fetch(`${apiBase}/localization/reset`, {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({ x: 0, y: 0, theta: 0 }),
      });
    } catch { /* best-effort */ }
  }, [demoMode, simEngine, onDemoReset, apiBase]);

  // ---------------------------------------------------------------------------
  // Render
  // ---------------------------------------------------------------------------

  const q = pose?.quality ?? 0;

  // In demo mode missionState is always 'idle' (WS suppressed).
  // Use activeWpIndex for waypoint nav, demoIsNavigating for other behaviors (e.g. aruco_seek).
  const isRunning  = demoMode ? (activeWpIndex >= 0 || demoIsNavigating) : missionState === 'running';
  const isComplete = !demoMode && missionState === 'completed';

  // Distance to active waypoint and rough ETA
  const MAX_V_MS = 0.30;  // m/s — matches sim/backend default
  const activeWp = isRunning && activeWpIndex >= 0 ? waypoints[activeWpIndex] ?? null : null;
  const distToWp = activeWp && pose
    ? Math.hypot(activeWp.x - pose.x, activeWp.y - pose.y)
    : null;
  const etaSec   = distToWp !== null ? distToWp / MAX_V_MS : null;

  const wpDone  = isRunning || isComplete ? Math.max(0, activeWpIndex) : 0;
  const wpTotal = waypoints.length;

  return (
    <div className="flex flex-col gap-3 text-white text-sm min-w-[240px]">

      {/* ── Sim backend launcher ──────────────────────────────────── */}
      <SimLauncherCard />

      {/* ── Connection status ─────────────────────────────────────── */}
      <div className="bg-gray-800 rounded-lg p-3 border border-gray-700">
        <div className="flex items-center justify-between mb-2">
          <span className="text-xs font-semibold uppercase tracking-wide text-gray-400">
            Pose Stream
          </span>
          <div className="flex items-center gap-1.5 text-xs">
            {statusDot(status)}
            <span className="text-gray-300 capitalize">{status}</span>
          </div>
        </div>

        {/* Pose readout */}
        <div className="grid grid-cols-3 gap-1 font-mono text-xs">
          <div className="bg-gray-900 rounded px-2 py-1.5 flex flex-col items-center">
            <span className="text-gray-500 text-[9px] mb-0.5">X</span>
            <span className="text-blue-300">{(pose?.x ?? 0).toFixed(3)}</span>
            <span className="text-gray-600 text-[9px]">m</span>
          </div>
          <div className="bg-gray-900 rounded px-2 py-1.5 flex flex-col items-center">
            <span className="text-gray-500 text-[9px] mb-0.5">Y</span>
            <span className="text-blue-300">{(pose?.y ?? 0).toFixed(3)}</span>
            <span className="text-gray-600 text-[9px]">m</span>
          </div>
          <div className="bg-gray-900 rounded px-2 py-1.5 flex flex-col items-center">
            <span className="text-gray-500 text-[9px] mb-0.5">θ</span>
            <span className="text-blue-300">
              {headingArrow(pose?.theta_deg ?? 0)}{(pose?.theta_deg ?? 0).toFixed(1)}°
            </span>
          </div>
        </div>
      </div>

      {/* ── EKF quality ───────────────────────────────────────────── */}
      <div className={`rounded-lg p-3 border ${qualityBgClass(q)}`}>
        <div className="flex items-center justify-between mb-1.5">
          <span className="text-xs font-semibold uppercase tracking-wide text-gray-400">
            EKF Localization
          </span>
          <span className={`text-xs font-bold ${qualityClass(q)}`}>
            {qualityLabel(q)}
          </span>
        </div>

        {/* Quality bar */}
        <div className="w-full h-1.5 bg-gray-700 rounded-full overflow-hidden mb-2">
          <div
            className="h-full rounded-full transition-all duration-300"
            style={{
              width: `${Math.round(q * 100)}%`,
              backgroundColor:
                q >= 0.7  ? '#34d399' :
                q >= 0.35 ? '#fbbf24' : '#f87171',
            }}
          />
        </div>

        <div className="flex justify-between text-[11px]">
          <span className="text-gray-400">
            Fixes: <span className={correctionCount > 0 ? 'text-emerald-400' : 'text-gray-500'}>
              {correctionCount}
            </span>
          </span>
          <span className="text-gray-400">
            Last marker: <span className="text-amber-300">
              {lastMarkerSeen != null ? `#${lastMarkerSeen}` : '—'}
            </span>
          </span>
        </div>
      </div>

      {/* ── Mission waypoints ─────────────────────────────────────── */}
      <div className="bg-gray-800 rounded-lg p-3 border border-gray-700">
        <div className="flex items-center justify-between mb-2">
          <span className="text-xs font-semibold uppercase tracking-wide text-gray-400">
            Waypoints
          </span>
          {wpTotal > 0 && (
            <span className="text-[11px] text-gray-400">
              {wpDone}/{wpTotal}
            </span>
          )}
        </div>

        {waypoints.length === 0 ? (
          <p className="text-gray-500 text-xs italic">
            Click the map to place waypoints
          </p>
        ) : (
          <div className="flex flex-col gap-1 max-h-[180px] overflow-y-auto pr-1">
            {waypoints.map((wp, i) => {
              const isDone   = (isRunning || isComplete) && i < activeWpIndex;
              const isActive = i === activeWpIndex && isRunning;
              return (
                <div
                  key={wp.id}
                  className={`flex items-center gap-2 rounded px-2 py-1 text-xs ${
                    isActive ? 'bg-emerald-900/40 border border-emerald-600/40' :
                    isDone   ? 'bg-gray-700/40 border border-gray-600/20' :
                               'bg-gray-900/40 border border-gray-700/40'
                  }`}
                >
                  <span className={`w-5 h-5 rounded-full flex items-center justify-center text-[10px] font-bold flex-shrink-0 ${
                    isActive ? 'bg-emerald-500 text-white' :
                    isDone   ? 'bg-gray-600 text-gray-400' :
                               'bg-gray-700 text-gray-300'
                  }`}>
                    {isDone ? '✓' : i + 1}
                  </span>
                  <span className={isDone ? 'text-gray-500 line-through' : 'text-gray-200'}>
                    {wp.label}
                  </span>
                  <span className="ml-auto text-gray-500 font-mono text-[10px]">
                    ({wp.x.toFixed(2)}, {wp.y.toFixed(2)})
                  </span>
                </div>
              );
            })}
          </div>
        )}

        {waypoints.length > 0 && (
          <button
            onClick={onClearWaypoints}
            className="mt-2 w-full text-[11px] text-gray-500 hover:text-red-400 transition-colors"
          >
            Clear all waypoints
          </button>
        )}
      </div>

      {/* ── Mission controls ──────────────────────────────────────── */}
      <div className="bg-gray-800 rounded-lg p-3 border border-gray-700 flex flex-col gap-2">
        <div className="flex items-center justify-between">
          <span className="text-xs font-semibold uppercase tracking-wide text-gray-400">
            Mission Control
          </span>
          {/* Mission stream connection dot */}
          <div className="flex items-center gap-1 text-[10px] text-gray-500">
            <span className={`w-1.5 h-1.5 rounded-full ${
              missionStreamConnected ? 'bg-emerald-400' : 'bg-gray-600 animate-pulse'
            }`} />
            <span>{missionStreamConnected ? 'stream ok' : 'stream off'}</span>
          </div>
        </div>

        {/* Status banners */}
        {isComplete && (
          <div className="bg-emerald-900/40 border border-emerald-600/40 rounded px-2 py-1.5 text-xs text-emerald-300 text-center font-semibold">
            ✓ Mission complete — all waypoints reached
          </div>
        )}
        {isRunning && (
          <div className="bg-blue-900/40 border border-blue-600/40 rounded px-2 py-1.5 text-xs text-blue-300 flex flex-col gap-1">
            <div className="flex items-center gap-2">
              <span className="w-2 h-2 rounded-full bg-blue-400 animate-pulse flex-shrink-0" />
              <span>
                {demoIsNavigating && activeWpIndex < 0
                  ? demoBehaviorLabel || 'Behavior active…'
                  : `Mission running — waypoint ${Math.max(0, activeWpIndex) + 1}/${waypoints.length}`}
              </span>
            </div>
            {distToWp !== null && (
              <div className="flex justify-between font-mono text-[10px] text-blue-400 pl-4">
                <span>dist: {distToWp.toFixed(2)} m</span>
                <span>ETA: ~{etaSec!.toFixed(0)} s</span>
              </div>
            )}
          </div>
        )}
        {/* Only show "waiting" banner in sim-backend mode (demo mode confirms instantly) */}
        {!demoMode && missionLaunched && missionState === 'idle' && (
          <div className="bg-amber-900/30 border border-amber-600/30 rounded px-2 py-1.5 text-xs text-amber-300 flex items-center gap-2">
            <span className="w-2 h-2 rounded-full bg-amber-400 animate-pulse flex-shrink-0" />
            <span>Waiting for sim to confirm start…</span>
          </div>
        )}

        <div className="flex gap-2">
          {!isRunning ? (
            <button
              onClick={startMission}
              disabled={waypoints.length === 0 || starting}
              className="flex-1 py-2 rounded bg-emerald-700 hover:bg-emerald-600 disabled:bg-gray-700 disabled:text-gray-500 text-white text-xs font-semibold transition-colors flex items-center justify-center gap-1.5"
            >
              {starting ? (
                <>
                  <span className="w-3 h-3 rounded-full border-2 border-white/30 border-t-white animate-spin" />
                  Starting…
                </>
              ) : isComplete ? (
                '↺ Run Again'
              ) : (
                '▶ Start Mission'
              )}
            </button>
          ) : (
            <button
              onClick={abortMission}
              className="flex-1 py-2 rounded bg-red-700 hover:bg-red-600 text-white text-xs font-semibold transition-colors"
            >
              ■ Abort
            </button>
          )}

          <button
            onClick={resetEkf}
            className="px-3 py-2 rounded bg-gray-700 hover:bg-gray-600 text-gray-300 text-xs font-semibold transition-colors"
            title="Reset EKF to origin"
          >
            ↺
          </button>
        </div>

        {missionError && (
          <p className="text-xs text-red-400 bg-red-900/20 rounded px-2 py-1">
            ⚠ {missionError}
          </p>
        )}
      </div>

      {/* ── Simulation quick-launch ───────────────────────────────── */}
      <div className="bg-gray-800 rounded-lg p-3 border border-indigo-700/40 flex flex-col gap-2">
        <div>
          <span className="text-xs font-semibold uppercase tracking-wide text-indigo-300">
            No Hardware? Start Here
          </span>
          <p className="text-[11px] text-gray-400 mt-1 leading-snug">
            This system runs fully in simulation. These scenarios use the same code that runs on real hardware.
          </p>
        </div>
        <div className="flex flex-col gap-1.5">
          {(
            [
              { name: 'marker_circuit',   icon: '⬡', label: 'Marker Circuit',   desc: '4 markers, loop path'     },
              { name: 'straight_line',    icon: '→', label: 'Straight Line',    desc: '2 markers, 3 m run'       },
              { name: 'three_point_turn', icon: '↙', label: 'Three Point Turn', desc: 'L-shaped maneuver'        },
            ] as const
          ).map(({ name, icon, label, desc }) => (
            <button
              key={name}
              onClick={() => loadScenario(name)}
              className="w-full py-1.5 rounded bg-indigo-900/60 hover:bg-indigo-800/60 border border-indigo-700/40 text-left px-3 transition-colors group"
            >
              <span className="text-indigo-300 text-xs">{icon} {label}</span>
              <span className="block text-[10px] text-gray-500 group-hover:text-gray-400">{desc}</span>
            </button>
          ))}
        </div>
        {simLoaded && (
          <p className="text-[11px] text-emerald-400">
            ✓ Scenario loaded — watch the map
          </p>
        )}
      </div>
    </div>
  );
};

export default MissionControlPanel;
