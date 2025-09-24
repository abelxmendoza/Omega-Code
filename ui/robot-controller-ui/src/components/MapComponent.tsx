/*
# File: /src/components/MapComponent.tsx
# Summary:
Simple map placeholder component (leaflet dependency removed for deployment)
- Shows a placeholder instead of actual map functionality
- Can be re-enabled later by adding leaflet dependency
*/

import React from 'react';

export interface MapComponentProps {
  className?: string;
  interactive?: boolean;
  wsUrl?: string;
  initialCenter?: [number, number];
  initialZoom?: number;
  showAccuracy?: boolean;
  dummy?: boolean;
  showTrail?: boolean;
}

const MapComponent: React.FC<MapComponentProps> = ({
  className = '',
  interactive = true,
  wsUrl,
  initialCenter = [0, 0],
  initialZoom = 13,
  showAccuracy = false,
  dummy = false,
  showTrail = false,
}) => {
  return (
    <div className={`w-full h-full bg-gray-800 flex items-center justify-center ${className}`}>
      <div className="text-center text-gray-400">
        <div className="text-4xl mb-2">🗺️</div>
        <div className="text-sm">Map Component</div>
        <div className="text-xs mt-1 opacity-75">
          {dummy ? 'Demo Mode' : 'GPS Disabled'}
        </div>
        {showAccuracy && (
          <div className="text-xs mt-1 opacity-50">
            Accuracy: ±5m
          </div>
        )}
      </div>
    </div>
  );
};

export default MapComponent;