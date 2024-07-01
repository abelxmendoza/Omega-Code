import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

// Fix for default marker icon issue with Webpack
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon-2x.png',
  iconUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png',
  shadowUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-shadow.png',
});

interface GpsLocationProps {
  size: 'small' | 'large';
  onSwitchToVideo: () => void;
}

const GpsLocation: React.FC<GpsLocationProps> = ({ size, onSwitchToVideo }) => {
  const [latitude, setLatitude] = useState<number>(37.7749); // Example latitude
  const [longitude, setLongitude] = useState<number>(-122.4194); // Example longitude

  useEffect(() => {
    // Simulate fetching GPS data
    const fetchGpsData = () => {
      // Replace this with actual GPS data fetching logic
      setLatitude(37.7749); // Example latitude
      setLongitude(-122.4194); // Example longitude
    };

    const intervalId = setInterval(fetchGpsData, 5000); // Fetch GPS data every 5 seconds

    return () => clearInterval(intervalId); // Cleanup interval on component unmount
  }, []);

  const mapStyle = size === 'small' ? { height: '100px', width: '100px' } : { height: '100%', width: '100%' };

  return (
    <div className="relative" style={mapStyle}>
      <MapContainer center={[latitude, longitude]} zoom={13} style={{ height: '100%', width: '100%' }}>
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        />
        <Marker position={[latitude, longitude]}>
          <Popup>
            Latitude: {latitude.toFixed(4)}, Longitude: {longitude.toFixed(4)}
          </Popup>
        </Marker>
      </MapContainer>
      {size === 'large' && (
        <div 
          className="absolute top-2 right-2 w-24 h-24 cursor-pointer"
          onClick={onSwitchToVideo}
          title="Switch to Video Feed"
        >
          <img 
            src="http://100.68.201.128:5000/video_feed" 
            alt="Video Feed" 
            className="w-full h-full object-cover" 
          />
        </div>
      )}
    </div>
  );
};

export default GpsLocation;
