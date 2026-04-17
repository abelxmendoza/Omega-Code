/**
 * /mission — Mission Control page.
 *
 * Layout:
 *   ┌──────────────────────────────────┬──────────────────┐
 *   │         MissionMap (canvas)      │ MissionControl   │
 *   │         fills left column        │ Panel (sidebar)  │
 *   └──────────────────────────────────┴──────────────────┘
 *
 * State ownership:
 *   - pose stream   → usePoseStream (WebSocket, 20 Hz)
 *   - waypoints     → useState here (lifted from MissionMap)
 *   - activeWpIndex → derived from useMissionStream WebSocket
 *   - markers       → pulled from /localization/status on mount
 *
 * This is a Pages Router page (not App Router) — consistent with the rest
 * of the project which uses Next.js 14 with pages/.
 */

import React, { useCallback, useEffect, useState } from 'react';
import Head from 'next/head';
import Link from 'next/link';
import dynamic from 'next/dynamic';

import { usePoseStream } from '@/hooks/usePoseStream';
import { useMissionStream } from '@/hooks/useMissionStream';
import type { Waypoint, MarkerDef } from '@/components/mission/MissionMap';
import MissionControlPanel from '@/components/mission/MissionControlPanel';

// MissionMap uses Canvas + requestAnimationFrame — disable SSR
const MissionMap = dynamic(
  () => import('@/components/mission/MissionMap'),
  { ssr: false, loading: () => <div className="w-full h-full bg-gray-950 rounded-lg animate-pulse" /> }
);

// ---------------------------------------------------------------------------
// Default marker map shown before loading scenario (empty)
// ---------------------------------------------------------------------------

const DEFAULT_MARKERS: MarkerDef[] = [];

// ---------------------------------------------------------------------------
// Scenario definitions — markers and waypoints that mirror the backend
// ---------------------------------------------------------------------------

const SCENARIO_MARKERS: Record<string, MarkerDef[]> = {
  marker_circuit: [
    { id: 0, x: 0.0, y: 0.0, alpha: 0.0 },
    { id: 1, x: 2.0, y: 0.0, alpha: Math.PI },
    { id: 2, x: 2.0, y: 2.0, alpha: Math.PI / 2 },
    { id: 3, x: 0.0, y: 2.0, alpha: -Math.PI / 2 },
  ],
  straight_line: [
    { id: 0, x: 0.0, y: 0.0, alpha: 0.0 },
    { id: 1, x: 3.0, y: 0.0, alpha: Math.PI },
  ],
  three_point_turn: [
    { id: 0, x: 0.0, y: 0.0, alpha: 0.0 },
    { id: 1, x: 2.0, y: 0.0, alpha: Math.PI / 2 },
    { id: 2, x: 2.0, y: 2.0, alpha: Math.PI },
  ],
};

const SCENARIO_WAYPOINTS: Record<string, Waypoint[]> = {
  marker_circuit: [
    { id: 1, x: 2.0, y: 0.0, label: 'WP1' },
    { id: 2, x: 2.0, y: 2.0, label: 'WP2' },
    { id: 3, x: 0.0, y: 2.0, label: 'WP3' },
    { id: 4, x: 0.0, y: 0.0, label: 'WP4' },
  ],
  straight_line: [
    { id: 1, x: 3.0, y: 0.0, label: 'WP1' },
    { id: 2, x: 0.0, y: 0.0, label: 'WP2' },
  ],
  three_point_turn: [
    { id: 1, x: 2.0, y: 0.0, label: 'WP1' },
    { id: 2, x: 2.0, y: 2.0, label: 'WP2' },
    { id: 3, x: 0.0, y: 0.0, label: 'WP3' },
  ],
};

// ---------------------------------------------------------------------------
// Page
// ---------------------------------------------------------------------------

export default function MissionPage() {
  const { pose, status, correctionCount, lastMarkerSeen, isConnected } =
    usePoseStream();

  const {
    waypointIndex: streamWpIndex,
    missionState,
    isConnected: missionConnected,
  } = useMissionStream();

  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [markers,   setMarkers]   = useState<MarkerDef[]>(DEFAULT_MARKERS);
  const [mapSize,   setMapSize]   = useState({ w: 600, h: 600 });

  // activeWpIndex drives both map highlighting and panel waypoint list.
  // -1 = no active waypoint (idle / no mission loaded).
  const activeWpIndex = missionState !== 'idle' ? streamWpIndex : -1;

  // ---------------------------------------------------------------------------
  // Responsive canvas size
  // ---------------------------------------------------------------------------

  useEffect(() => {
    function measure() {
      // Sidebar is ~280px + gap ~16px; keep canvas square
      const available = Math.min(
        window.innerWidth  - 320,
        window.innerHeight - 140,
      );
      const size = Math.max(360, Math.min(available, 800));
      setMapSize({ w: size, h: size });
    }
    measure();
    window.addEventListener('resize', measure);
    return () => window.removeEventListener('resize', measure);
  }, []);

  // ---------------------------------------------------------------------------
  // Scenario loaded callback — update markers and waypoints on the map
  // ---------------------------------------------------------------------------

  const handleScenarioLoaded = useCallback((scenarioName: string) => {
    const m = SCENARIO_MARKERS[scenarioName];
    if (m) setMarkers(m);
    const wps = SCENARIO_WAYPOINTS[scenarioName];
    if (wps) setWaypoints(wps);
  }, []);

  // ---------------------------------------------------------------------------
  // Waypoint management
  // ---------------------------------------------------------------------------

  const handleClearWaypoints = useCallback(() => {
    setWaypoints([]);
  }, []);

  // ---------------------------------------------------------------------------
  // Render
  // ---------------------------------------------------------------------------

  return (
    <>
      <Head>
        <title>Mission Control — Omega-1</title>
      </Head>

      <div className="min-h-screen bg-gray-900 text-white flex flex-col">

        {/* ── Top bar ─────────────────────────────────────────────── */}
        <header className="flex items-center justify-between px-4 py-3 bg-gray-800 border-b border-gray-700 flex-shrink-0">
          <div className="flex items-center gap-3">
            <Link href="/" className="text-gray-400 hover:text-white text-sm transition-colors">
              ← Dashboard
            </Link>
            <span className="text-gray-600">|</span>
            <h1 className="text-sm font-bold tracking-wide text-white">
              Ω OMEGA-1 · Mission Control
            </h1>
          </div>

          <div className="flex items-center gap-3 text-xs">
            {/* EKF quality pill */}
            <div className={`flex items-center gap-1.5 px-2 py-1 rounded border font-mono ${
              (pose?.quality ?? 0) >= 0.7  ? 'bg-emerald-900/40 border-emerald-600/40 text-emerald-400' :
              (pose?.quality ?? 0) >= 0.35 ? 'bg-amber-900/40 border-amber-600/40 text-amber-400' :
                                              'bg-red-900/40 border-red-600/40 text-red-400'
            }`}>
              <span>EKF</span>
              <span>{((pose?.quality ?? 0) * 100).toFixed(0)}%</span>
            </div>

            {/* WS status pill */}
            <div className={`flex items-center gap-1.5 px-2 py-1 rounded border text-xs ${
              isConnected
                ? 'bg-blue-900/40 border-blue-600/40 text-blue-300'
                : 'bg-gray-800 border-gray-600 text-gray-500'
            }`}>
              <span className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-blue-400' : 'bg-gray-500 animate-pulse'}`} />
              <span>{isConnected ? '/ws/pose' : 'connecting…'}</span>
            </div>
          </div>
        </header>

        {/* ── Main content ─────────────────────────────────────────── */}
        <main className="flex-1 flex gap-4 p-4 overflow-hidden">

          {/* LEFT: Mission map */}
          <div className="flex-1 flex items-start justify-center">
            <MissionMap
              pose={pose}
              markers={markers}
              waypoints={waypoints}
              onWaypointsChange={setWaypoints}
              activeWpIndex={activeWpIndex}
              width={mapSize.w}
              height={mapSize.h}
              ppm={80}
            />
          </div>

          {/* RIGHT: Control panel */}
          <aside className="w-[268px] flex-shrink-0 overflow-y-auto pb-4">
            <MissionControlPanel
              pose={pose}
              status={status}
              correctionCount={correctionCount}
              lastMarkerSeen={lastMarkerSeen}
              waypoints={waypoints}
              activeWpIndex={activeWpIndex}
              missionState={missionState}
              missionStreamConnected={missionConnected}
              onClearWaypoints={handleClearWaypoints}
              onScenarioLoaded={handleScenarioLoaded}
            />
          </aside>
        </main>

        {/* ── Bottom status bar ────────────────────────────────────── */}
        <footer className="flex items-center justify-between px-4 py-2 bg-gray-800 border-t border-gray-700 flex-shrink-0 text-[11px] text-gray-500">
          <span>
            Pose: ({(pose?.x ?? 0).toFixed(3)}, {(pose?.y ?? 0).toFixed(3)}) m
            · θ {(pose?.theta_deg ?? 0).toFixed(1)}°
            · Fixes: {correctionCount}
          </span>
          <span>
            {markers.length > 0 && `${markers.length} markers · `}
            {waypoints.length} waypoints
          </span>
        </footer>
      </div>
    </>
  );
}
