import React from 'react';
import { useCommandLog } from './CommandLogContext';

const CommandLog: React.FC = () => {
  const { commands } = useCommandLog();

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
