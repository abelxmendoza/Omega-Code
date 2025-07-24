/**
 * File: src/components/sensors/LineTrackerStatus.tsx
 * Summary:
 *   Real-time status panel for the robot's line tracking sensor (3 IR sensors).
 *   Connects to a WebSocket server (URL from .env only) and displays IR01, IR02, IR03 state.
 *   Compatible with both `{ sensors: { left, center, right } }` or `{ IR01, IR02, IR03 }` JSON payloads.
 */

import React, { useState, useEffect, useRef } from 'react';

// Prefer Tailscale, fallback to LAN. Never hardcode your own IP!
const LINE_TRACKER_WS =
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN;

const LineTrackerStatus: React.FC = () => {
  const [data, setData] = useState({ IR01: 0, IR02: 0, IR03: 0 });
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (!LINE_TRACKER_WS) return;

    ws.current = new WebSocket(LINE_TRACKER_WS);

    ws.current.onopen = () => {
      console.log('[LINE TRACKER] WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      try {
        const result = JSON.parse(event.data);
        if (result.sensors) {
          // Accept shape: { sensors: { left, center, right } }
          setData({
            IR01: result.sensors.left ?? 0,
            IR02: result.sensors.center ?? 0,
            IR03: result.sensors.right ?? 0,
          });
        } else if (
          result.IR01 !== undefined &&
          result.IR02 !== undefined &&
          result.IR03 !== undefined
        ) {
          setData({
            IR01: result.IR01,
            IR02: result.IR02,
            IR03: result.IR03,
          });
        }
      } catch (err) {
        // Ignore invalid JSON
      }
    };

    ws.current.onclose = () => {
      console.log('[LINE TRACKER] WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('[LINE TRACKER] WebSocket error:', error);
    };

    return () => {
      ws.current?.close();
    };
  }, []);

  return (
    <div className="bg-gray-900 text-white p-4 rounded-lg shadow-md my-2">
      <h3 className="text-md font-semibold mb-2 underline">Line Tracking Status</h3>
      <p>IR01: {data.IR01}</p>
      <p>IR02: {data.IR02}</p>
      <p>IR03: {data.IR03}</p>
    </div>
  );
};

export default LineTrackerStatus;
