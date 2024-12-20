/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/MapComponent.tsx
# Summary:
This component renders an interactive map using the Leaflet library. It includes features to display the robot's location on the map in real-time using WebSocket data. The component initializes a map, adds a marker with a custom robot icon, and updates the marker's position based on incoming WebSocket messages.
*/

import React, { useEffect, useRef } from 'react';
import L from 'leaflet'; // Leaflet library for maps
import 'leaflet/dist/leaflet.css'; // Leaflet default styles

// Define the MapComponent functional component
const MapComponent: React.FC = () => {
  const mapRef = useRef<L.Map | null>(null); // Ref to store the Leaflet map instance
  const ws = useRef<WebSocket | null>(null); // Ref to store the WebSocket instance

  useEffect(() => {
    // Initialize the map with default view and zoom level
    mapRef.current = L.map('map').setView([51.505, -0.09], 13);

    // Add OpenStreetMap tile layer
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(mapRef.current);

    // Create a custom icon for the robot marker
    const robotIcon = L.divIcon({
      html: '🤖',
      className: 'custom-robot-icon', // Custom CSS class for styling
      iconSize: [30, 30], // Size of the icon
      iconAnchor: [15, 15], // Anchor point of the icon
      popupAnchor: [0, -15] // Popup position relative to the icon
    });

    // Add a marker to the map with the custom icon
    const marker = L.marker([51.505, -0.09], { icon: robotIcon }).addTo(mapRef.current)
      .bindPopup('A pretty CSS3 popup.<br> Easily customizable.')
      .openPopup();

    // Establish WebSocket connection to receive real-time data
    ws.current = new WebSocket('ws://localhost:8080/ws');

    // WebSocket event handlers
    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      // Update marker position based on location data
      if (data.type === 'location') {
        const { lat, lng } = data;
        marker.setLatLng([lat, lng]);
        mapRef.current?.setView([lat, lng], 13); // Center map on new location
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Cleanup function to remove the map and close the WebSocket on unmount
    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
      }
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []); // Empty dependency array to ensure this effect runs only once

  // Render the map container
  return <div id="map" className="w-full h-full" style={{ height: '100%', width: '100%' }}></div>;
};

export default MapComponent;

/*
## Steps to Resolve TypeScript Error:
The error indicates missing type declarations for the `leaflet` module. To fix this, you must install the corresponding type declarations.

1. Install the Leaflet type definitions:
```bash
npm install --save-dev @types/leaflet
```

2. Restart your development server and ensure TypeScript recognizes the `leaflet` types.

3. Ensure your `tsconfig.json` includes the following to locate type declarations:
```json
{
  "compilerOptions": {
    "typeRoots": ["./node_modules/@types"]
  }
}
```

These steps ensure that TypeScript correctly handles the `leaflet` library.
*/
