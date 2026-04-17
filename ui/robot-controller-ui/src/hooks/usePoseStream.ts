/**
 * usePoseStream — subscribes to /ws/pose and returns live SE(2) EKF pose data.
 *
 * Built on top of the existing useRobustWebSocket infrastructure so reconnect
 * logic, error handling, and status tracking are already covered.
 *
 * Usage:
 *   const { pose, status, correctionCount, lastMarkerSeen } = usePoseStream();
 *
 * The hook derives its WebSocket URL from the gateway config (same host/port
 * as the FastAPI backend) so it automatically respects network profiles
 * (local/lan/tailscale) without extra configuration.
 *
 * pose is null until the first frame arrives.  Components should treat null
 * as "no fix yet" and display a placeholder.
 */

'use client';

import { useCallback, useRef, useState } from 'react';
import { useRobustWebSocket } from '@/utils/RobustWebSocket';
import { buildGatewayUrl } from '@/config/gateway';

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
  ts:               number;   // monotonic timestamp from server
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
// URL builder — converts http(s) gateway base to ws(s)
// ---------------------------------------------------------------------------

function buildPoseWsUrl(): string {
  // buildGatewayUrl returns something like http://localhost:8000
  const http = buildGatewayUrl('/ws/pose');
  return http.replace(/^http/, 'ws');
}

// ---------------------------------------------------------------------------
// Hook
// ---------------------------------------------------------------------------

export function usePoseStream(): UsePoseStreamResult {
  const [pose, setPose] = useState<PoseData | null>(null);
  const [correctionCount, setCorrectionCount] = useState(0);
  const [lastMarkerSeen, setLastMarkerSeen] = useState<number | null>(null);

  // Build URL once per mount — gateway config is static
  const wsUrl = useRef(buildPoseWsUrl()).current;

  const handleMessage = useCallback((data: unknown) => {
    if (!data || typeof data !== 'object') return;
    const msg = data as Record<string, unknown>;

    if (msg.type === 'pose') {
      setPose({
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

  const { isConnected, connectionStatus } = useRobustWebSocket({
    url:                  wsUrl,
    reconnectInterval:    2000,
    maxReconnectAttempts: 20,   // keep trying — sim or robot may restart
    onMessage:            handleMessage,
  });

  return {
    pose,
    status:          connectionStatus as PoseStreamStatus,
    correctionCount,
    lastMarkerSeen,
    isConnected,
  };
}
