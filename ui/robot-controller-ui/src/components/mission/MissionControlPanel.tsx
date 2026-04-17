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
}) => {
  const [missionError, setMissionError] = useState<string | null>(null);
  const [simLoaded, setSimLoaded]       = useState(false);

  const apiBase = buildGatewayUrl('');   // e.g. http://localhost:8000

  // ---------------------------------------------------------------------------
  // Actions
  // ---------------------------------------------------------------------------

  const startMission = useCallback(async () => {
    if (waypoints.length === 0) {
      setMissionError('Add at least one waypoint on the map first.');
      return;
    }
    setMissionError(null);

    try {
      // Step 1 — Ensure the sim loop is running.
      // POST /sim/start always succeeds: it (re)creates the robot at origin
      // and resets the EKF, giving a clean starting state for manual missions.
      const startRes = await fetch(`${apiBase}/sim/start`, {
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

      // Step 2 — Load the waypoint mission
      const res = await fetch(`${apiBase}/sim/mission`, {
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
      // missionState → 'running' when mission_started arrives on /ws/mission stream
    } catch (err) {
      setMissionError(err instanceof Error ? err.message : String(err));
    }
  }, [waypoints, apiBase]);

  const abortMission = useCallback(async () => {
    try {
      await fetch(`${apiBase}/sim/mission/abort`, { method: 'POST' });
      // State transitions to 'idle' when backend emits mission_aborted
    } catch { /* best-effort */ }
  }, [apiBase]);

  const loadScenario = useCallback(async (name: string) => {
    setMissionError(null);
    try {
      const res = await fetch(`${apiBase}/sim/scenario`, {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({ name }),
      });
      if (!res.ok) {
        if (res.status === 404) {
          throw new Error('Backend not in sim mode — restart with SIM_MODE=1');
        }
        throw new Error(`${res.status}: ${await res.text()}`);
      }
      setSimLoaded(true);
      onScenarioLoaded(name);  // parent updates markers + waypoints on the map
      // missionState → 'running' when mission_started arrives on /ws/mission stream
    } catch (err) {
      setMissionError(err instanceof Error ? err.message : String(err));
    }
  }, [apiBase, onScenarioLoaded]);

  const resetEkf = useCallback(async () => {
    try {
      await fetch(`${apiBase}/localization/reset`, {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({ x: 0, y: 0, theta: 0 }),
      });
    } catch { /* best-effort */ }
  }, [apiBase]);

  // ---------------------------------------------------------------------------
  // Render
  // ---------------------------------------------------------------------------

  const q        = pose?.quality ?? 0;
  const wpDone   = missionState !== 'idle' ? Math.max(0, activeWpIndex) : 0;
  const wpTotal  = waypoints.length;

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
              const isDone   = missionState !== 'idle' && i < activeWpIndex;
              const isActive = i === activeWpIndex && missionState === 'running';
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

        {/* Mission complete banner */}
        {missionState === 'completed' && (
          <div className="bg-emerald-900/40 border border-emerald-600/40 rounded px-2 py-1.5 text-xs text-emerald-300 text-center font-semibold">
            Mission complete — all waypoints reached
          </div>
        )}

        <div className="flex gap-2">
          {missionState !== 'running' ? (
            <button
              onClick={startMission}
              disabled={waypoints.length === 0}
              className="flex-1 py-2 rounded bg-emerald-700 hover:bg-emerald-600 disabled:bg-gray-700 disabled:text-gray-500 text-white text-xs font-semibold transition-colors"
            >
              {missionState === 'completed' ? '↺ Run Again' : '▶ Start Mission'}
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
            {missionError}
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
