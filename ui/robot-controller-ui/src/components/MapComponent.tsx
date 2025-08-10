/*
# File: /src/components/MapComponent.tsx
# Summary:
Leaflet map that shows the robot’s GPS with a live WebSocket update.
- Uses env-resolved WS: NEXT_PUBLIC_BACKEND_WS_URL_LOCATION_*
- Accepts many payload shapes: {lat,lng} | {latitude,longitude} | {location:{lat,lng}} | {gps:{lat,lon}}
- Tiny status dot in header (connecting/connected/disconnected)
*/

import React, { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { resolveWsUrl } from '@/utils/resolveWsUrl';

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

function StatusDot({ status }: { status: ServerStatus }) {
  const color =
    status === 'connected' ? 'bg-emerald-500'
      : status === 'connecting' ? 'bg-slate-500'
      : 'bg-rose-500';
  return <span className={`inline-block rounded-full ${color}`} style={{ width: 8, height: 8 }} />;
}

function parseLatLng(raw: any): { lat: number; lng: number } | null {
  if (!raw) return null;
  if (typeof raw.lat === 'number' && typeof raw.lng === 'number') return { lat: raw.lat, lng: raw.lng };
  if (typeof raw.latitude === 'number' && typeof raw.longitude === 'number') return { lat: raw.latitude, lng: raw.longitude };
  if (raw.location) return parseLatLng(raw.location);
  if (raw.gps) {
    const g = raw.gps;
    if (typeof g.lat === 'number' && typeof g.lon === 'number') return { lat: g.lat, lng: g.lon };
  }
  return null;
}

export default function MapComponent({
  initialCenter = [51.505, -0.09] as [number, number],
  zoom = 13,
  className = '',
  title = 'GPS',
}: {
  initialCenter?: [number, number];
  zoom?: number;
  className?: string;
  title?: string;
}) {
  const mapEl = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<L.Map | null>(null);
  const markerRef = useRef<L.Marker | null>(null);
  const wsRef = useRef<WebSocket | null>(null);

  const [status, setStatus] = useState<ServerStatus>('connecting');

  // Resolve location WS from env
  const LOCATION_WS = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION');

  useEffect(() => {
    if (!mapEl.current) return;

    // Init map
    const map = L.map(mapEl.current, {
      zoomControl: false,
      attributionControl: false,
    }).setView(initialCenter, zoom);
    mapRef.current = map;

    // Tiles
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
    }).addTo(map);

    // Robot marker (emoji to avoid icon assets)
    const robotIcon = L.divIcon({
      html: '🤖',
      className: 'custom-robot-icon',
      iconSize: [24, 24],
      iconAnchor: [12, 12],
    });

    markerRef.current = L.marker(initialCenter, { icon: robotIcon }).addTo(map);

    return () => {
      map.remove();
      mapRef.current = null;
      markerRef.current = null;
    };
  }, [initialCenter, zoom]);

  useEffect(() => {
    if (!LOCATION_WS) {
      setStatus('disconnected');
      return;
    }
    let cancelled = false;
    let retry: ReturnType<typeof setTimeout> | null = null;

    const open = (attempt = 0) => {
      if (cancelled) return;
      setStatus('connecting');
      let ws: WebSocket;
      try {
        ws = new WebSocket(LOCATION_WS);
      } catch {
        const backoff = Math.min(1000 * 2 ** attempt, 10000);
        retry = setTimeout(() => open(attempt + 1), backoff);
        return;
      }
      wsRef.current = ws;

      ws.onopen = () => setStatus('connected');
      ws.onclose = () => {
        setStatus('disconnected');
        const backoff = Math.min(1000 * 2 ** attempt, 10000);
        retry = setTimeout(() => open(attempt + 1), backoff);
      };
      ws.onerror = () => {/* onclose handles state */};

      ws.onmessage = (evt) => {
        try {
          const data = JSON.parse(evt.data);
          const p = parseLatLng(data);
          if (!p || !mapRef.current || !markerRef.current) return;
          markerRef.current.setLatLng([p.lat, p.lng]);
          // Smooth recenter a bit
          mapRef.current.setView([p.lat, p.lng], mapRef.current.getZoom(), { animate: true });
        } catch {
          /* ignore */
        }
      };
    };

    open(0);
    return () => {
      cancelled = true;
      if (retry) clearTimeout(retry);
      try { wsRef.current?.close(); } catch {}
      wsRef.current = null;
    };
  }, [LOCATION_WS]);

  return (
    <div className={`rounded-lg shadow-md overflow-hidden border border-white/10 bg-gray-900 ${className}`}>
      <div className="flex items-center justify-between px-3 py-2 bg-black/40 border-b border-white/10">
        <div className="flex items-center gap-2 text-white/90">
          <StatusDot status={status} />
          <span className="text-sm font-semibold">{title}</span>
          <span className="text-xs text-white/70 ml-2">
            {status === 'connected' ? 'Live' : status === 'connecting' ? 'Connecting…' : 'Disconnected'}
          </span>
        </div>
      </div>
      <div ref={mapEl} className="w-full h-44" />
    </div>
  );
}
