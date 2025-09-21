/*
# File: /src/components/GpsLocation.tsx
# Summary:
Next.js-friendly wrapper around MapComponent (Leaflet) with SSR disabled.
Pass `dummy` to simulate movement when you don't have a GPS server yet.
*/

import dynamic from 'next/dynamic';
import React from 'react';

const MapComponent = dynamic(() => import('./MapComponent'), { ssr: false });

type LatLng = [number, number];

export default function GpsLocation({
  className = '',
  interactive = true,
  wsUrl,
  initialCenter = [37.7749, -122.4194] as LatLng,
  initialZoom = 15,
  showAccuracy = false,
  dummy = false,
  showTrail = true,
}: {
  className?: string;
  interactive?: boolean;
  wsUrl?: string;
  initialCenter?: LatLng;
  initialZoom?: number;
  showAccuracy?: boolean;
  dummy?: boolean;
  showTrail?: boolean;
}) {
  return (
    <div className={`w-full h-full ${className}`}>
      <h2 className="text-white font-bold mb-4">GPS Location</h2>
      <MapComponent
        interactive={interactive}
        wsUrl={wsUrl}
        initialCenter={initialCenter}
        initialZoom={initialZoom}
        showAccuracy={showAccuracy}
        dummy={dummy}
        showTrail={showTrail}
      />
    </div>
  );
}
