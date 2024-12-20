/*
# File: /src/components/VideoFeed.tsx
# Summary:
This component displays a video feed from a specified URL.
It toggles between the video stream and GPS location view using a button.
*/

import React, { useState, useEffect, useRef } from 'react';
import Image from 'next/image';
import GpsLocation from './GpsLocation';

const VideoFeed: React.FC = () => {
  const [showGps, setShowGps] = useState(false);
  const ws = useRef<WebSocket | null>(null);
  const wsUrl = process.env.NEXT_PUBLIC_BACKEND_WS_URL || 'ws://localhost:8080/ws';

  useEffect(() => {
    ws.current = new WebSocket(wsUrl);

    ws.current.onopen = () => console.log('WebSocket connection established');
    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      console.log('WebSocket message received:', data);
    };
    ws.current.onclose = () => console.log('WebSocket connection closed');
    ws.current.onerror = (error) => console.error('WebSocket error:', error);

    return () => ws.current?.close();
  }, [wsUrl]);

  const toggleView = () => setShowGps((prev) => !prev);

  return (
    <div className="relative w-2/5 bg-gray-200 flex items-center justify-center" style={{ height: 'calc(60vw * 0.6)' }}>
      {showGps ? (
        <div className="absolute top-0 right-0 w-full h-full">
          <GpsLocation />
          <button
            className="absolute top-2 right-2 bg-white text-black p-2 rounded shadow"
            onClick={toggleView}
          >
            Back to Video
          </button>
        </div>
      ) : (
        <>
          <Image
            src="http://100.68.201.128:5000/video_feed"
            alt="Video Feed"
            layout="fill"
            objectFit="cover"
            priority
          />
          <div
            className="absolute top-2 right-2 w-24 h-24 cursor-pointer"
            onClick={toggleView}
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
