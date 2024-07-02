import React, { useEffect, useRef } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

const MapComponent: React.FC = () => {
  const mapRef = useRef<L.Map | null>(null);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Initialize the map
    mapRef.current = L.map('map').setView([51.505, -0.09], 13);

    // Add a tile layer to the map
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(mapRef.current);

    // Create a custom icon using a robot emoji
    const robotIcon = L.divIcon({
      html: '🤖',
      className: 'custom-robot-icon',
      iconSize: [30, 30],
      iconAnchor: [15, 15],
      popupAnchor: [0, -15]
    });

    // Add a marker to the map
    const marker = L.marker([51.505, -0.09], { icon: robotIcon }).addTo(mapRef.current)
      .bindPopup('A pretty CSS3 popup.<br> Easily customizable.')
      .openPopup();

    // Initialize WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'location') {
        const { lat, lng } = data;
        marker.setLatLng([lat, lng]);
        mapRef.current?.setView([lat, lng], 13);
      }
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    // Cleanup on unmount
    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
      }
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  return <div id="map" className="w-full h-full" style={{ height: '100%', width: '100%' }}></div>;
};

export default MapComponent;