/**
 * usePoseStream — SE(2) EKF pose, with Demo Mode interception.
 *
 * LIVE mode:  subscribes to /ws/pose WebSocket (existing behaviour).
 * DEMO mode:  subscribes to the SimEngine in DemoModeContext instead.
 *             No network connections are made; pose is synthesized locally.
 *
 * All consumers (LocalizationPanel, MissionMap, mission.tsx) are unaffected —
 * the returned interface is identical in both modes.
 */

'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { useRobustWebSocket } from '@/utils/RobustWebSocket';
import { buildGatewayUrl } from '@/config/gateway';
import { useDemoMode } from '@/context/DemoModeContext';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

export interface PoseData {
  x:                number;   // metres, world frame
  y:                number;   // metres, world frame
  theta_rad:        number;   // radians, [-π, π]
  theta_deg:        number;   // degrees
  quality:          number;   // 0–1 EKF confidence
  correction_count: number;   // total ArUco fixes applied
  last_marker_seen: number | null;
  ts:               number;   // monotonic timestamp
}

export type PoseStreamStatus = 'connecting' | 'connected' | 'disconnected' | 'error';

export interface UsePoseStreamResult {
  pose:             PoseData | null;
  status:           PoseStreamStatus;
  correctionCount:  number;
  lastMarkerSeen:   number | null;
  isConnected:      boolean;
}

// ---------------------------------------------------------------------------
// URL builder
// ---------------------------------------------------------------------------

function buildPoseWsUrl(): string {
  const http = buildGatewayUrl('/ws/pose');
  return http.replace(/^http/, 'ws');
}

// ---------------------------------------------------------------------------
// Hook
// ---------------------------------------------------------------------------

export function usePoseStream(): UsePoseStreamResult {
  const { demoMode, engine, isHydrated } = useDemoMode();

  // ── Demo path ──────────────────────────────────────────────────────────────
  const [demoPose, setDemoPose] = useState<PoseData | null>(null);

  useEffect(() => {
    if (!demoMode) {
      setDemoPose(null);
      return;
    }
    const unsub = engine.subscribePose((p) => {
      setDemoPose({
        x:                p.x,
        y:                p.y,
        theta_rad:        p.theta_rad,
        theta_deg:        p.theta_rad * (180 / Math.PI),
        quality:          0.85,   // simulated EKF confidence
        correction_count: 0,
        last_marker_seen: null,
        ts:               Date.now(),
      });
    });
    return unsub;
  }, [demoMode, engine]);

  // ── Live WS path ───────────────────────────────────────────────────────────
  // Always called (satisfies React hook rules). Disabled in demo mode by
  // passing an empty URL — useRobustWebSocket returns 'error' status and
  // never opens a socket when the URL is falsy.
  const [livePose, setLivePose] = useState<PoseData | null>(null);
  const [correctionCount, setCorrectionCount] = useState(0);
  const [lastMarkerSeen, setLastMarkerSeen]   = useState<number | null>(null);

  const wsUrl = useRef(buildPoseWsUrl()).current;

  const handleMessage = useCallback((data: unknown) => {
    if (!data || typeof data !== 'object') return;
    const msg = data as Record<string, unknown>;

    if (msg.type === 'pose') {
      setLivePose({
        x:                Number(msg.x          ?? 0),
        y:                Number(msg.y          ?? 0),
        theta_rad:        Number(msg.theta_rad  ?? 0),
        theta_deg:        Number(msg.theta_deg  ?? 0),
        quality:          Number(msg.quality    ?? 0),
        correction_count: Number(msg.correction_count ?? 0),
        last_marker_seen: msg.last_marker_seen != null ? Number(msg.last_marker_seen) : null,
        ts:               Number(msg.ts         ?? 0),
      });
      setCorrectionCount(Number(msg.correction_count ?? 0));
      setLastMarkerSeen(msg.last_marker_seen != null ? Number(msg.last_marker_seen) : null);
    }
  }, []);

  // Suppress WS until localStorage is read (isHydrated) and demo mode is resolved.
  // Passing '' to useRobustWebSocket prevents any connection attempt.
  const suppress = !isHydrated || demoMode;

  const { isConnected, connectionStatus } = useRobustWebSocket({
    url:                  suppress ? '' : wsUrl,
    reconnectInterval:    2000,
    maxReconnectAttempts: suppress ? 0 : 20,
    onMessage:            handleMessage,
  });

  // ── Return ─────────────────────────────────────────────────────────────────

  if (!isHydrated) {
    return { pose: null, status: 'connecting', correctionCount: 0, lastMarkerSeen: null, isConnected: false };
  }

  if (demoMode) {
    return {
      pose:            demoPose,
      status:          'connected',
      correctionCount: 0,
      lastMarkerSeen:  null,
      isConnected:     true,
    };
  }

  return {
    pose:            livePose,
    status:          connectionStatus as PoseStreamStatus,
    correctionCount,
    lastMarkerSeen,
    isConnected,
  };
}
