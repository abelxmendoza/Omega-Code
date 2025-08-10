/*
# File: /src/components/VideoFeed.tsx
# Summary:
Live MJPEG video with a mini GPS map (top-right). Clicking the mini map expands it.
Endpoints are chosen by NEXT_PUBLIC_NETWORK_PROFILE. Includes stream availability
checks and a generic WS auto-reconnect (optional).
*/

import React, { useState, useEffect, useRef } from 'react';
import GpsLocation from './GpsLocation';

/** Resolve endpoint from profile: lan | tailscale | local */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local';
  return (
    process.env[`${base}_${profile.toUpperCase()}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
};

// Pull endpoints from env
const wsUrl = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL');            // optional generic WS
const videoUrl = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL');       // MJPEG stream

const VideoFeed: React.FC = () => {
  const [mapExpanded, setMapExpanded] = useState(false);
  const [videoAvailable, setVideoAvailable] = useState(true);
  const ws = useRef<WebSocket | null>(null);

  // Check the MJPEG stream with HEAD so we can show "not connected"
  const checkVideoStream = async () => {
    if (!videoUrl) {
      setVideoAvailable(false);
      return;
    }
    try {
      const res = await fetch(videoUrl, { method: 'HEAD', cache: 'no-store' });
      setVideoAvailable(res.ok);
    } catch {
      setVideoAvailable(false);
    }
  };

  useEffect(() => {
    checkVideoStream();
    const id = setInterval(checkVideoStream, 5000);
    return () => clearInterval(id);
  }, [videoUrl]);

  // Optional generic WS (kept from your original; safe to remove if unused)
  useEffect(() => {
    if (!wsUrl) return;
    const connect = () => {
      ws.current = new WebSocket(wsUrl);
      ws.current.onopen = () => console.log('✅ VideoFeed WS connected');
      ws.current.onmessage = (e) => console.log('📡', e.data);
      ws.current.onclose = () => setTimeout(connect, 5000);
      ws.current.onerror = (err) => console.error('🚨 WS error:', err);
    };
    connect();
    return () => ws.current?.close();
  }, [wsUrl]);

  return (
    <div
      className="relative w-2/5 bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden"
      style={{ height: 'calc(60vw * 0.6)' }}
    >
      {/* Video or unavailable placeholder */}
      {videoAvailable ? (
        <img
          src={videoUrl}
          alt="Live Video Feed"
          className="absolute inset-0 w-full h-full object-cover"
          onError={() => setVideoAvailable(false)}
        />
      ) : (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-800 text-white/85">
          <span className="px-3 py-1 rounded-full text-xs font-semibold bg-rose-600/20 border border-rose-500/40">
            Video feed not connected
          </span>
        </div>
      )}

      {/* Mini GPS map (top-right) */}
      {!mapExpanded && (
        <button
          type="button"
          className="absolute top-2 right-2 z-10 w-24 h-24 sm:w-28 sm:h-28 rounded-md overflow-hidden border border-white/15 shadow-md bg-black/30 hover:bg-black/40 transition"
          onClick={() => setMapExpanded(true)}
          title="Open GPS map"
        >
          {/* Disable map interactions while mini to avoid hijacking scroll/drag */}
          <div className="pointer-events-none w-full h-full">
            <GpsLocation />
          </div>
        </button>
      )}

      {/* Expanded full-frame map overlay */}
      {mapExpanded && (
        <div className="absolute inset-0 z-20 bg-black/70 backdrop-blur-sm">
          <div className="absolute inset-3 rounded-lg overflow-hidden border border-white/15 shadow-lg bg-gray-900">
            <GpsLocation />
            <button
              type="button"
              className="absolute top-2 right-2 bg-black/60 hover:bg-black/70 text-white text-xs px-2 py-1 rounded border border-white/20"
              onClick={() => setMapExpanded(false)}
            >
              Back to Video
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default VideoFeed;
