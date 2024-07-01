// File: /Omega-Code/ui/robot-controller-ui/src/components/VideoFeed.tsx

/*
This component displays a video feed from a specified URL.
It renders the video stream inside a div container with predefined styles for width and height.
*/

import React, { useState } from 'react';
import GpsLocation from './GpsLocation';

const VideoFeed: React.FC = () => {
  const [showGps, setShowGps] = useState(false);

  const toggleView = () => {
    setShowGps(!showGps);
  };

  return (
    <div className="relative w-2/5 bg-gray-200 flex items-center justify-center" style={{ height: 'calc(60vw * 0.6)' }}>
      {showGps ? (
        <div className="absolute top-0 right-0 w-full h-full">
          <GpsLocation size="large" onSwitchToVideo={toggleView} />
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
            <GpsLocation size="small" onSwitchToVideo={toggleView} />
          </div>
        </>
      )}
    </div>
  );
};

export default VideoFeed;
