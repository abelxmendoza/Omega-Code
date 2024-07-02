import React, { useEffect, useRef } from 'react';
import { useCommandLog } from './CommandLogContext';

const CommandLog: React.FC = () => {
  const { commands = [], addCommand } = useCommandLog(); // Default to an empty array if commands is undefined
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    // Establish WebSocket connection
    ws.current = new WebSocket('ws://localhost:8080/ws');

    ws.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.command) {
        addCommand(data.command);
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
  }, [addCommand]);

  console.log('Rendering CommandLog with commands:', commands);

  return (
    <div className="w-full md:w-3/4 lg:w-1/2 p-4 bg-gray-900 rounded-lg shadow-md">
      <h2 className="text-lg font-bold text-green-400 border-b-2 border-green-400 pb-2">Command Log</h2>
      <ul className="mt-4 h-20 overflow-y-auto bg-gray-800 rounded-lg p-2">
        {commands.map((command, index) => (
          <li key={index} className="text-center text-green-300">{command}</li>
        ))}
      </ul>
    </div>
  );
};

export default CommandLog;
