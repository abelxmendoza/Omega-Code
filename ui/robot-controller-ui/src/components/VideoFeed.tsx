/*
# File: /src/components/VideoFeed.tsx
# Summary:
Live MJPEG video with a mini GPS map (top-right). Clicking the mini map expands it.
Endpoints are chosen by NEXT_PUBLIC_NETWORK_PROFILE. Includes stream availability
checks, a tiny status dot with HEAD latency, and a generic WS auto-reconnect (optional).
*/

import React, { useState, useEffect, useRef } from 'react';
import GpsLocation from './GpsLocation';

/** Resolve endpoint from profile: lan | tailscale | local */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local';
  return (
    (process.env as any)[`${base}_${profile.toUpperCase()}`] ||
    (process.env as any)[`${base}_LOCAL`] ||
    ''
  );
};

// Pull endpoints from env
const wsUrl = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL');            // optional generic WS
const videoUrl = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL');       // MJPEG stream

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

function StatusDot({ status, title }: { status: ServerStatus; title: string }) {
  const color =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'connecting'
      ? 'bg-slate-500'
      : 'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color}`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
}

const VideoFeed: React.FC = () => {
  const [mapExpanded, setMapExpanded] = useState(false);
  const [videoAvailable, setVideoAvailable] = useState(true);

  const [status, setStatus] = useState<ServerStatus>('connecting');
  const [pingMs, setPingMs] = useState<number | null>(null);

  const ws = useRef<WebSocket | null>(null);

  // Check the MJPEG stream with HEAD so we can show "not connected" + latency
  const checkVideoStream = async () => {
    if (!videoUrl) {
      setVideoAvailable(false);
      setStatus('disconnected');
      setPingMs(null);
      return;
    }
    try {
      setStatus((s) => (s === 'connected' ? s : 'connecting'));
      const start = performance.now();
      const res = await fetch(videoUrl, { method: 'HEAD', cache: 'no-store' });
      const end = performance.now();
      if (res.ok) {
        setVideoAvailable(true);
        setStatus('connected');
        setPingMs(Math.max(0, Math.round(end - start)));
      } else {
        setVideoAvailable(false);
        setStatus('disconnected');
        setPingMs(null);
      }
    } catch {
      setVideoAvailable(false);
      setStatus('disconnected');
      setPingMs(null);
    }
  };

  useEffect(() => {
    checkVideoStream();
    const id = setInterval(checkVideoStream, 5000);
    return () => clearInterval(id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
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

  const statusTitle =
    `Video: ${status[0].toUpperCase()}${status.slice(1)}${pingMs != null ? ` • ${pingMs}ms` : ''}`;

  return (
    <div
      className="relative w-2/5 bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden"
      style={{ height: 'calc(60vw * 0.6)' }}
    >
      {/* Tiny status pill (top-left) */}
      <div className="absolute top-2 left-2 z-20 flex items-center gap-2 px-2 py-1 bg-black/45 backdrop-blur rounded border border-white/15 text-white text-xs">
        <StatusDot status={status} title={statusTitle} />
        <span className="font-semibold">Video</span>
        <span className="text-white/80">{pingMs != null ? `${pingMs}ms` : '… ms'}</span>
      </div>

      {/* Video or unavailable placeholder */}
      {videoAvailable ? (
        <img
          src={videoUrl}
          alt="Live Video Feed"
          className="absolute inset-0 w-full h-full object-cover"
          onLoad={() => setStatus('connected')}
          onError={() => {
            setVideoAvailable(false);
            setStatus('disconnected');
          }}
        />
      ) : (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-800 text-white/85">
          <span className="px-3 py-1 rounded-full text-xs font-semibold bg-rose-600/20 border border-rose-500/40">
            Video feed not connected
          </span>
        </div>
      )}

      {/* Mini GPS map (top-right, dummy) */}
      {!mapExpanded && (
        <button
          type="button"
          className="absolute top-2 right-2 z-10 w-24 h-24 sm:w-28 sm:h-28 rounded-md overflow-hidden border border-white/15 shadow-md bg-black/30 hover:bg-black/40 transition"
          onClick={() => setMapExpanded(true)}
          title="Open GPS map"
        >
          {/* Disable interactions in mini view to avoid hijacking page scroll/drag */}
          <div className="pointer-events-none w-full h-full">
            <GpsLocation interactive={false} dummy showTrail={false} />
          </div>
        </button>
      )}

      {/* Expanded full-frame map overlay (still dummy) */}
      {mapExpanded && (
        <div className="absolute inset-0 z-20 bg-black/70 backdrop-blur-sm">
          <div className="absolute inset-3 rounded-lg overflow-hidden border border-white/15 shadow-lg bg-gray-900">
            <GpsLocation interactive dummy showAccuracy />
            <div className="absolute top-2 left-2 bg-amber-500/80 text-black text-xs font-semibold px-2 py-1 rounded">
              GPS (demo)
            </div>
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
