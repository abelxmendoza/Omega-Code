// File: /Omega-Code/ui/robot-controller-ui/src/components/VideoFeed.tsx

/*
This component displays a video feed from a specified URL.
It renders the video stream inside a div container with predefined styles for width and height.
*/

import React from 'react';

const VideoFeed: React.FC = () => {
  return (
    <div className="w-2/5 bg-gray-200 flex items-center justify-center" style={{ height: 'calc(60vw * 0.6)' }}>
      {/* Display the video feed from the specified URL */}
      <img 
        src="http://100.68.201.128:5000/video_feed" 
        alt="Video Feed" 
        className="w-full h-full object-cover" 
      />
    </div>
  );
};

export default VideoFeed;
