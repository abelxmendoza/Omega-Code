import React, { useState, useEffect } from 'react';

const UltrasonicSensorStatus: React.FC = () => {
    const [distance, setDistance] = useState(0);

    useEffect(() => {
        const fetchData = async () => {
            const response = await fetch('http://localhost:8080/ultrasonic-sensor');
            const result = await response.json();
            setDistance(result.distance);
        };
        fetchData();
    }, []);

    return (
        <div>
            <h3>Ultrasonic Sensor Status</h3>
            <p>Distance: {distance} cm</p>
        </div>
    );
};

export default UltrasonicSensorStatus;
