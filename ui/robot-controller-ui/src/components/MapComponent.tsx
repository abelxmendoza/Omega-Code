/*
# File: /src/components/MapComponent.tsx
# Summary:
Leaflet map with optional dummy mode (no WS) to simulate movement.
- Dummy mode animates a circular path with heading arrow and an optional trail.
- Works inside tiny PiP tiles; can be interactive:false to avoid scroll/zoom.
*/

import React, { useEffect, useRef } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

export interface MapComponentProps {
  className?: string;
  interactive?: boolean;
  wsUrl?: string;                 // reserved for real GPS later
  initialCenter?: [number, number];
  initialZoom?: number;
  showAccuracy?: boolean;
  dummy?: boolean;                // when true (or no wsUrl), simulate movement
  showTrail?: boolean;            // show breadcrumb polyline
  trailLength?: number;           // max trail points
  demoSpeedMs?: number;           // tick interval in ms
  demoRadiusDeg?: number;         // circle radius (in degrees)
}

const MapComponent: React.FC<MapComponentProps> = ({
  className = '',
  interactive = true,
  wsUrl,
  initialCenter = [37.7749, -122.4194], // SF default
  initialZoom = 15,
  showAccuracy = false,
  dummy = false,
  showTrail = true,
  trailLength = 80,
  demoSpeedMs = 150,
  demoRadiusDeg = 0.0007, // ~70m
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const markerRef = useRef<L.Marker | null>(null);
  const trailRef = useRef<L.Polyline | null>(null);
  const accuracyRef = useRef<L.Circle | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const demoTimer = useRef<number | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // Create the map (NOTE: no 'tap' option; not in MapOptions in newer Leaflet types)
    const map = L.map(containerRef.current, {
      zoomControl: interactive,
      dragging: interactive,
      scrollWheelZoom: interactive,
      doubleClickZoom: interactive,
      boxZoom: interactive,
      keyboard: interactive,
      attributionControl: false,
    }).setView(initialCenter, initialZoom);
    mapRef.current = map;

    // Disable legacy tap handler if present and not interactive (older Leaflet/mobile handlers)
    const anyMap = map as any;
    if (!interactive && anyMap?.tap?.disable) {
      anyMap.tap.disable();
    }

    // Tiles
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
    }).addTo(map);

    // Heading arrow marker (DivIcon so we can rotate)
    const arrow = L.divIcon({
      className: 'robot-heading-icon',
      html: `<div style="
        width:18px;height:18px;
        transform: translate(-50%,-50%);
        display:flex;align-items:center;justify-content:center;">
        <svg viewBox="0 0 24 24" width="18" height="18" fill="white">
          <path d="M12 2l4 10-4-2-4 2 4-10z"/>
        </svg>
      </div>`,
      iconSize: [18, 18],
      iconAnchor: [9, 9],
    });

    const marker = L.marker(initialCenter, { icon: arrow }).addTo(map);
    markerRef.current = marker;

    // Optional accuracy ring (static in dummy)
    if (showAccuracy) {
      const c = L.circle(initialCenter, { radius: 10, color: '#3b82f6', fillOpacity: 0.1 }).addTo(map);
      accuracyRef.current = c;
    }

    // Optional trail
    if (showTrail) {
      trailRef.current = L.polyline([initialCenter], { color: '#22d3ee', opacity: 0.8, weight: 2 }).addTo(map);
    }

    // --- DUMMY MODE (or when wsUrl missing): animate a tiny loop ---
    if (dummy || !wsUrl) {
      let t = 0;
      const center = L.latLng(initialCenter[0], initialCenter[1]);
      demoTimer.current = window.setInterval(() => {
        t += 0.08;
        const lat = center.lat + demoRadiusDeg * Math.sin(t);
        const lng = center.lng + demoRadiusDeg * Math.cos(t);
        const pos = L.latLng(lat, lng);

        // Update marker position
        marker.setLatLng(pos);

        // Rotate arrow to heading (derivative on the circle)
        const headingRad = Math.atan2(
          demoRadiusDeg * Math.cos(t),
          -demoRadiusDeg * Math.sin(t)
        );
        const deg = (headingRad * 180) / Math.PI;
        const el = marker.getElement();
        if (el) el.style.transform = `translate(-50%,-50%) rotate(${deg}deg)`;

        // Trail
        if (trailRef.current) {
          const latlngs = (trailRef.current.getLatLngs() as L.LatLng[]).concat([pos]);
          if (latlngs.length > trailLength) latlngs.splice(0, latlngs.length - trailLength);
          trailRef.current.setLatLngs(latlngs);
        }

        // Keep it centered without jerky zoom
        map.panTo(pos, { animate: true });
      }, Math.max(80, demoSpeedMs));
    } else {
      // --- Real WS mode (placeholder) ---
      try {
        wsRef.current = new WebSocket(wsUrl);
        wsRef.current.onmessage = (evt) => {
          try {
            const msg = JSON.parse(evt.data);
            // Expect { type: 'location', lat, lng, accuracy? }
            if (msg?.type === 'location' && typeof msg.lat === 'number' && typeof msg.lng === 'number') {
              const pos = L.latLng(msg.lat, msg.lng);
              marker.setLatLng(pos);
              if (trailRef.current) {
                const latlngs = (trailRef.current.getLatLngs() as L.LatLng[]).concat([pos]);
                if (latlngs.length > trailLength) latlngs.splice(0, latlngs.length - trailLength);
                trailRef.current.setLatLngs(latlngs);
              }
              if (accuracyRef.current && typeof msg.accuracy === 'number') {
                accuracyRef.current.setLatLng(pos).setRadius(msg.accuracy);
              }
              map.panTo(pos, { animate: true });
            }
          } catch { /* ignore */ }
        };
      } catch { /* ws optional */ }
    }

    // Cleanup
    return () => {
      if (demoTimer.current) clearInterval(demoTimer.current);
      try { wsRef.current?.close(); } catch {}
      map.remove();
      mapRef.current = null;
    };
  }, [
    interactive,
    wsUrl,
    initialCenter,
    initialZoom,
    showAccuracy,
    dummy,
    showTrail,
    trailLength,
    demoSpeedMs,
    demoRadiusDeg,
  ]);

  // Prevent scroll from "grabbing" the page when map is interactive:false
  const pointerClasses = interactive ? '' : 'pointer-events-none';

  return (
    <div
      ref={containerRef}
      className={`${className} ${pointerClasses}`}
      style={{ width: '100%', height: '100%' }}
    />
  );
};

export default MapComponent;
