// File: /Omega-Code/ui/robot-controller-ui/src/components/VideoFeed.tsx

/*
This component displays a video feed from a specified URL.
It renders the video stream inside a div container with predefined styles for width and height.
*/

import React, { useState, useEffect, useRef } from 'react';
import GpsLocation from './GpsLocation';

const VideoFeed: React.FC = () => {
  const [showGps, setShowGps] = useState(false);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      // Handle incoming WebSocket messages if needed
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Cleanup on unmount
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  const toggleView = () => {
    setShowGps(!showGps);
  };

  return (
    <div className="relative w-2/5 bg-gray-200 flex items-center justify-center" style={{ height: 'calc(60vw * 0.6)' }}>
      {showGps ? (
        <div className="absolute top-0 right-0 w-full h-full">
          <GpsLocation />
          <button 
            className="absolute top-2 right-2 bg-white p-2 rounded"
            onClick={toggleView}
          >
            Back to Video
          </button>
        </div>
      ) : (
        <>
          <img 
            src="http://100.68.201.128:5000/video_feed" 
            alt="Video Feed" 
            className="w-full h-full object-cover" 
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

