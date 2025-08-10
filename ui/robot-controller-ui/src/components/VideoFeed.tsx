/*
# File: /src/components/VideoFeed.tsx
# Summary:
Displays a live MJPEG video stream and toggles to GPS view. 
Video and WebSocket endpoints are selected from environment variables 
based on your current network profile (LAN, Tailscale, or Localhost) 
to avoid leaking real IP addresses. Handles video availability checks, 
auto-reconnects WebSocket, and keeps your UI safe for sharing.
*/

import React, { useState, useEffect, useRef } from 'react';
import GpsLocation from './GpsLocation';

/**
 * Utility to select the proper endpoint for the current network profile.
 * 
 * Usage: set NEXT_PUBLIC_NETWORK_PROFILE in .env.local to 'lan', 'tailscale', or 'local'
 * and define *_LAN, *_TAILSCALE, *_LOCAL endpoints.
 */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local'; // fallback to local
  const envVar =
    process.env[`${base}_${profile.toUpperCase()}`] ||
    process.env[`${base}_LOCAL`] ||
    '';
  return envVar;
};

// Pull endpoints from env, with safe defaults
const wsUrl = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL');
const videoUrl = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL');

const VideoFeed: React.FC = () => {
  const [showGps, setShowGps] = useState(false);
  const [videoAvailable, setVideoAvailable] = useState(true);
  const ws = useRef<WebSocket | null>(null);

  /**
   * Checks if the MJPEG video stream is available before rendering.
   * Prevents broken stream preview in the UI.
   */
  const checkVideoStream = async () => {
    if (!videoUrl) {
      setVideoAvailable(false);
      return;
    }
    try {
      const response = await fetch(videoUrl, { method: 'HEAD' });
      setVideoAvailable(response.ok);
    } catch (error) {
      console.error('❌ Error checking video stream:', error);
      setVideoAvailable(false);
    }
  };

  // Check video stream availability on mount and poll every 5s
  useEffect(() => {
    checkVideoStream();
    const interval = setInterval(checkVideoStream, 5000);
    return () => clearInterval(interval);
  }, [videoUrl]);

  /**
   * WebSocket logic for real-time robot updates (if needed).
   * Will reconnect automatically if closed.
   */
  useEffect(() => {
    if (!wsUrl) return; // skip if missing

    const connectWebSocket = () => {
      ws.current = new WebSocket(wsUrl);

      ws.current.onopen = () =>
        console.log('✅ WebSocket connection established');
      ws.current.onmessage = (event) =>
        console.log('📡 WebSocket message received:', event.data);
      ws.current.onclose = () => {
        console.warn('⚠️ WebSocket closed, reconnecting in 5s...');
        setTimeout(connectWebSocket, 5000);
      };
      ws.current.onerror = (error) =>
        console.error('🚨 WebSocket error:', error);
    };

    connectWebSocket();
    return () => ws.current?.close();
  }, [wsUrl]);

  return (
    <div
      className="relative w-2/5 bg-gray-200 flex items-center justify-center"
      style={{ height: 'calc(60vw * 0.6)' }}
    >
      {showGps ? (
        <div className="absolute top-0 right-0 w-full h-full">
          <GpsLocation />
          <button
            className="absolute top-2 right-2 bg-white text-black p-2 rounded shadow"
            onClick={() => setShowGps(false)}
          >
            Back to Video
          </button>
        </div>
      ) : (
        <>
          {videoAvailable ? (
            <img
              src={videoUrl}
              alt="Live Video Feed"
              className="w-full h-full object-cover"
              onError={() => setVideoAvailable(false)}
            />
          ) : (
            <div className="absolute inset-0 flex items-center justify-center bg-gray-300 text-gray-700">
              <p>⚠️ Video feed unavailable</p>
            </div>
          )}
          <div
            className="absolute top-2 right-2 w-24 h-24 cursor-pointer"
            onClick={() => setShowGps(true)}
            title="Click to enlarge"
          >
            <GpsLocation />
          </div>
        </>
      )}
    </div>
  );
};

export default VideoFeed;
