import React, { useState, useEffect, useRef } from 'react';

const UltrasonicSensorStatus: React.FC = () => {
    const [distance, setDistance] = useState(0);
    const ws = useRef<WebSocket | null>(null);

    useEffect(() => {
        // Establish WebSocket connection
        ws.current = new WebSocket('ws://localhost:8080/ws');

        ws.current.onopen = () => {
            console.log('WebSocket connection established');
        };

        ws.current.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.distance !== undefined) {
                setDistance(data.distance);
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
            if (ws.current) {
                ws.current.close();
            }
        };
    }, []);

    return (
        <div>
            <h3>Ultrasonic Sensor Status</h3>
            <p>Distance: {distance} cm</p>
        </div>
    );
};

export default UltrasonicSensorStatus;

