/**
 * useMissionStream — subscribes to /ws/mission and tracks mission state.
 *
 * The hook is the single source of truth for:
 *   - which waypoint the robot is currently targeting (waypointIndex)
 *   - whether a mission is idle / running / completed
 *
 * State machine:
 *   idle ──mission_started──► running ──mission_completed──► completed
 *    ▲                             │
 *    └────────mission_aborted──────┘
 *
 * On reconnect the server sends a `mission_status` snapshot so the UI
 * always reflects the real backend state without needing to poll.
 */

'use client';

import { useCallback, useRef, useState } from 'react';
import { useRobustWebSocket } from '@/utils/RobustWebSocket';
import { buildGatewayUrl } from '@/config/gateway';
import { useDemoMode } from '@/context/DemoModeContext';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

export type MissionState = 'idle' | 'running' | 'completed';

export interface MissionEvent {
  type:             'mission_event';
  event:
    | 'mission_status'
    | 'mission_started'
    | 'waypoint_reached'
    | 'mission_completed'
    | 'mission_aborted';
  waypoint_index:   number;
  waypoints_total?: number;
  mission_active?:  boolean;
  mission_complete?: boolean;
  timestamp:        number;
}

export interface UseMissionStreamResult {
  /** Index of the current target waypoint (0-based). -1 when idle. */
  waypointIndex:  number;
  /** Total number of waypoints in the active mission. */
  waypointsTotal: number;
  missionState:   MissionState;
  lastEvent:      MissionEvent | null;
  isConnected:    boolean;
}

// ---------------------------------------------------------------------------
// URL builder
// ---------------------------------------------------------------------------

function buildMissionWsUrl(): string {
  const http = buildGatewayUrl('/ws/mission');
  return http.replace(/^http/, 'ws');
}

// ---------------------------------------------------------------------------
// Hook
// ---------------------------------------------------------------------------

export function useMissionStream(overrideUrl?: string): UseMissionStreamResult {
  const { demoMode, simBackendMode, isHydrated } = useDemoMode();
  const [waypointIndex,  setWaypointIndex]  = useState<number>(-1);
  const [waypointsTotal, setWaypointsTotal] = useState<number>(0);
  const [missionState,   setMissionState]   = useState<MissionState>('idle');
  const [lastEvent,      setLastEvent]      = useState<MissionEvent | null>(null);

  // Priority: explicit override → sim-backend auto → gateway (Pi)
  const gatewayUrl = useRef(buildMissionWsUrl()).current;
  const wsUrl = overrideUrl ?? (simBackendMode ? 'ws://localhost:8000/ws/mission' : gatewayUrl);

  const handleMessage = useCallback((data: unknown) => {
    if (!data || typeof data !== 'object') return;
    const msg = data as Record<string, unknown>;

    if (msg.type !== 'mission_event') return;

    const evt: MissionEvent = {
      type:             'mission_event',
      event:            msg.event as MissionEvent['event'],
      waypoint_index:   Number(msg.waypoint_index  ?? -1),
      waypoints_total:  msg.waypoints_total != null ? Number(msg.waypoints_total) : undefined,
      mission_active:   msg.mission_active  != null ? Boolean(msg.mission_active) : undefined,
      mission_complete: msg.mission_complete != null ? Boolean(msg.mission_complete) : undefined,
      timestamp:        Number(msg.timestamp ?? 0),
    };

    setLastEvent(evt);
    if (evt.waypoints_total != null) setWaypointsTotal(evt.waypoints_total);

    switch (evt.event) {
      case 'mission_status':
        // Sync from backend on (re)connect
        if (evt.mission_complete) {
          setMissionState('completed');
          setWaypointIndex(evt.waypoints_total ?? 0);
        } else if (evt.mission_active) {
          setMissionState('running');
          setWaypointIndex(evt.waypoint_index);
        } else {
          setMissionState('idle');
          setWaypointIndex(-1);
        }
        break;

      case 'mission_started':
        setMissionState('running');
        setWaypointIndex(0);
        break;

      case 'waypoint_reached':
        // waypoint_index is the index just completed; next target = index + 1
        setWaypointIndex(evt.waypoint_index + 1);
        break;

      case 'mission_completed':
        setMissionState('completed');
        setWaypointIndex(evt.waypoints_total ?? evt.waypoint_index);
        break;

      case 'mission_aborted':
        setMissionState('idle');
        setWaypointIndex(-1);
        break;
    }
  }, []);

  const suppress = !isHydrated || demoMode;

  const { isConnected } = useRobustWebSocket({
    url:                  suppress ? '' : wsUrl,
    reconnectInterval:    2000,
    maxReconnectAttempts: suppress ? 0 : 20,
    onMessage:            handleMessage,
  });

  if (!isHydrated || demoMode) {
    return { waypointIndex: -1, waypointsTotal: 0, missionState: 'idle', lastEvent: null, isConnected: demoMode };
  }

  return { waypointIndex, waypointsTotal, missionState, lastEvent, isConnected };
}
