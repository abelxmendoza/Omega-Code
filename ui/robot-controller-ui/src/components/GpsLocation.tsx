import dynamic from 'next/dynamic';
import React from 'react';

// Dynamically import the component that uses Leaflet
const MapComponent = dynamic(() => import('./MapComponent'), { ssr: false });

const GpsLocation = () => {
  return (
    <div>
      <h1>GPS Location</h1>
      <MapComponent />
    </div>
  );
};

export default GpsLocation;
