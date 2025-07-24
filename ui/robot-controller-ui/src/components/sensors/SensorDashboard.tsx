// File: src/components/sensors/SensorDashboard.tsx

/**
 * SensorDashboard Component
 * 
 * Displays real-time sensor data received over WebSocket:
 * - Line Tracking Sensor (IR01, IR02, IR03)
 * - Ultrasonic Sensor (distance in cm, m, inches, feet)
 * 
 * Tries Tailscale first, falls back to LAN. Works with both
 * separate or combined servers for line tracking & ultrasonic.
 */

import React, { useEffect, useState, useRef } from 'react';

interface LineTrackingData {
  IR01: number;
  IR02: number;
  IR03: number;
}
interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

// ------------- WebSocket URLs (from env) ----------------
const LINE_TRACKER_WS =
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_TAILSCALE ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER_LAN;

const ULTRASONIC_WS =
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_TAILSCALE ||
  process.env.NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LAN;

const SensorDashboard: React.FC = () => {
  // --- State ---
  const [lineTrackingData, setLineTrackingData] = useState<LineTrackingData>({
    IR01: 0, IR02: 0, IR03: 0,
  });
  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0, distance_m: 0, distance_inch: 0, distance_feet: 0,
  });

  // --- WebSocket refs ---
  const lineWs = useRef<WebSocket | null>(null);
  const ultrasonicWs = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (!LINE_TRACKER_WS) return;
    lineWs.current = new WebSocket(LINE_TRACKER_WS);
    lineWs.current.onopen = () => console.log(`[LINE] WebSocket connected: ${LINE_TRACKER_WS}`);
    lineWs.current.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        // Accept both 'sensors' and raw fields
        if (data.sensors) {
          // { sensors: { left, center, right } }
          setLineTrackingData({
            IR01: data.sensors.left ?? 0,
            IR02: data.sensors.center ?? 0,
            IR03: data.sensors.right ?? 0,
          });
        } else if (data.IR01 !== undefined) {
          // { IR01, IR02, IR03 }
          setLineTrackingData({
            IR01: data.IR01,
            IR02: data.IR02,
            IR03: data.IR03,
          });
        }
      } catch (e) {
        // Ignore invalid
      }
    };
    lineWs.current.onerror = (e) => console.warn('[LINE] WebSocket error:', e);
    lineWs.current.onclose = () => console.log('[LINE] WebSocket closed');

    return () => lineWs.current?.close();
  }, []);

  useEffect(() => {
    if (!ULTRASONIC_WS) return;
    ultrasonicWs.current = new WebSocket(ULTRASONIC_WS);
    ultrasonicWs.current.onopen = () => console.log(`[ULTRASONIC] WebSocket connected: ${ULTRASONIC_WS}`);
    ultrasonicWs.current.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.distance_cm !== undefined) {
          setUltrasonicData({
            distance_cm: data.distance_cm,
            distance_m: data.distance_m,
            distance_inch: data.distance_inch,
            distance_feet: data.distance_feet,
          });
        }
      } catch (e) {
        // Ignore invalid
      }
    };
    ultrasonicWs.current.onerror = (e) => console.warn('[ULTRASONIC] WebSocket error:', e);
    ultrasonicWs.current.onclose = () => console.log('[ULTRASONIC] WebSocket closed');

    return () => ultrasonicWs.current?.close();
  }, []);

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md max-w-6xl mx-auto">
      <h2 className="text-lg font-bold mb-4 text-center">Sensor Dashboard</h2>
      <div className="sensor-container grid grid-cols-1 sm:grid-cols-2 gap-6">
        {/* Line Tracking */}
        <div className="line-tracking bg-gray-900 p-4 rounded-lg shadow-lg">
          <strong className="block text-sm mb-2 underline">Line Tracking Sensor</strong>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>
        {/* Ultrasonic */}
        <div className="ultrasonic-distance bg-gray-900 p-4 rounded-lg shadow-lg">
          <strong className="block text-sm mb-2 underline">Ultrasonic Distance</strong>
          <p>{ultrasonicData.distance_cm} cm</p>
          <p>{ultrasonicData.distance_m.toFixed(2)} m</p>
          <p>{ultrasonicData.distance_inch.toFixed(2)} inches</p>
          <p>{ultrasonicData.distance_feet.toFixed(2)} feet</p>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;
