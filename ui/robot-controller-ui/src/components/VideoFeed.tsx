/*
# File: /src/components/VideoFeed.tsx
# Summary:
This component displays a live MJPEG video feed from a specified URL.
It toggles between the video stream and GPS location view using a button.
Additionally, it maintains a WebSocket connection for real-time updates and
includes error handling for video availability.
*/

import React, { useState, useEffect, useRef } from 'react';
import GpsLocation from './GpsLocation';

const VideoFeed: React.FC = () => {
  const [showGps, setShowGps] = useState(false);
  const [videoAvailable, setVideoAvailable] = useState(true);
  const ws = useRef<WebSocket | null>(null);

  // Define WebSocket and Video Stream URLs
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://100.68.201.128:8080/ws';
  const videoUrl = 'https://192.168.1.134:5000/video_feed';

  /**
   * Function to check if the video stream is available.
   * This prevents the UI from displaying a broken video stream.
   */
  const checkVideoStream = async () => {
    try {
      const response = await fetch(videoUrl, { method: 'HEAD' });
      setVideoAvailable(response.ok);
    } catch (error) {
      console.error('❌ Error checking video stream:', error);
      setVideoAvailable(false);
    }
  };

  useEffect(() => {
    checkVideoStream(); // Initial check when component mounts
    const interval = setInterval(checkVideoStream, 5000); // Re-check every 5 seconds
    return () => clearInterval(interval); // Cleanup on unmount
  }, []);

  /**
   * Establishes and maintains a WebSocket connection.
   * Automatically attempts reconnection if the connection closes.
   */
  useEffect(() => {
    const connectWebSocket = () => {
      ws.current = new WebSocket(wsUrl);

      ws.current.onopen = () => console.log('✅ WebSocket connection established');
      ws.current.onmessage = (event) => console.log('📡 WebSocket message received:', JSON.parse(event.data));
      ws.current.onclose = () => {
        console.warn('⚠️ WebSocket connection closed, attempting to reconnect...');
        setTimeout(connectWebSocket, 5000); // Retry connection after 5 seconds
      };
      ws.current.onerror = (error) => console.error('🚨 WebSocket error:', error);
    };

    connectWebSocket();
    return () => ws.current?.close(); // Cleanup WebSocket on unmount
  }, [wsUrl]);

  return (
    <div className="relative w-2/5 bg-gray-200 flex items-center justify-center" style={{ height: 'calc(60vw * 0.6)' }}>
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
              onError={() => setVideoAvailable(false)} // Hide if an error occurs
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
