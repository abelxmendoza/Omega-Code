// File: /src/components/GpsLocation.tsx
// Summary:
// Lightweight wrapper around MapComponent with dynamic import (no SSR).
// - Resolves the location WS URL from NEXT_PUBLIC_NETWORK_PROFILE
// - Lets callers toggle interactivity (great for PiP overlays)
// - Pass-through props for initial center/zoom and accuracy display

import React from 'react';
import dynamic from 'next/dynamic';

type LatLng = [number, number];

export interface GpsLocationProps {
  className?: string;
  interactive?: boolean;          // default true; set false for PiP overlay
  wsUrl?: string;                 // override; otherwise resolved from env
  initialCenter?: LatLng;         // default [51.505, -0.09]
  initialZoom?: number;           // default 13
  showAccuracy?: boolean;         // optional visual accuracy circle
}

/** Resolve endpoint from profile: lan | tailscale | local */
const getEnvVar = (base: string) => {
  const profile = process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local';
  return (
    process.env[`${base}_${profile.toUpperCase()}`] ||
    process.env[`${base}_LOCAL`] ||
    ''
  );
};

// Prefer env-resolved WS for location updates
const defaultLocationWs = getEnvVar('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION');

// Dynamically import the Leaflet map (client only)
type MapProps = GpsLocationProps;
const MapComponent = dynamic<MapProps>(() => import('./MapComponent'), {
  ssr: false,
  loading: () => (
    <div className="w-full h-full flex items-center justify-center text-xs text-white/70 bg-black/20">
      Loading map…
    </div>
  ),
});

const GpsLocation: React.FC<GpsLocationProps> = ({
  className = '',
  interactive = true,
  wsUrl,
  initialCenter = [51.505, -0.09],
  initialZoom = 13,
  showAccuracy,
}) => {
  const resolvedWs = wsUrl || defaultLocationWs;

  // When used as a tiny overlay, disabling pointer events keeps the video scroll/drag intact.
  const pointerClass = interactive ? '' : 'pointer-events-none';

  return (
    <div className={`w-full h-full ${pointerClass} ${className}`}>
      <MapComponent
        interactive={interactive}
        wsUrl={resolvedWs}
        initialCenter={initialCenter}
        initialZoom={initialZoom}
        showAccuracy={showAccuracy}
      />
    </div>
  );
};

export default GpsLocation;
