import React, { useState, useEffect } from 'react';

const LineTrackerStatus: React.FC = () => {
    const [data, setData] = useState({ ir01: 0, ir02: 0, ir03: 0 });

    useEffect(() => {
        const fetchData = async () => {
            const response = await fetch('http://localhost:8080/line-tracking');
            const result = await response.json();
            setData(result);
        };
        fetchData();
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

