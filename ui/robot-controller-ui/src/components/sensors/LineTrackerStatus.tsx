import React, { useState, useEffect, useRef } from 'react';

const LineTrackerStatus: React.FC = () => {
    const [data, setData] = useState({ ir01: 0, ir02: 0, ir03: 0 });
    const ws = useRef<WebSocket | null>(null);

    useEffect(() => {
        // Establish WebSocket connection
        ws.current = new WebSocket('ws://localhost:8080/ws');

        ws.current.onopen = () => {
            console.log('WebSocket connection established');
        };

        ws.current.onmessage = (event) => {
            const result = JSON.parse(event.data);
            if (result.ir01 !== undefined && result.ir02 !== undefined && result.ir03 !== undefined) {
                setData(result);
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
            <h3>Line Tracking Status</h3>
            <p>IR01: {data.ir01}</p>
            <p>IR02: {data.ir02}</p>
            <p>IR03: {data.ir03}</p>
        </div>
    );
};

export default LineTrackerStatus;

