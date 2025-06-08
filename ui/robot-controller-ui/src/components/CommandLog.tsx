/*
# File: /src/components/CommandLog.tsx
# Summary:
A component that displays a real-time log of commands sent or received.
Commands are managed centrally via the CommandContext and include timestamps.
This component provides a scrollable list with user-friendly formatting and interaction.
*/

import React from 'react';
import { useCommand } from '../context/CommandContext';

const CommandLog: React.FC = () => {
  const { commands } = useCommand(); // Access the command log from CommandContext

  return (
    <div className="w-full md:w-3/4 lg:w-1/2 p-4 bg-gray-900 rounded-lg shadow-md">
      {/* Header */}
      <h2 className="text-lg font-bold text-green-400 border-b-2 border-green-400 pb-2">
        Command Log
      </h2>

      {/* Command List */}
      <ul className="mt-4 h-40 overflow-y-auto bg-gray-800 rounded-lg p-2">
        {commands.length > 0 ? (
          // Render the list of commands
          commands.map((entry, index) => (
            <li
              key={index} // Unique key for React rendering
              className="text-green-300 py-1 text-sm hover:text-green-200 transition-colors cursor-pointer flex items-start"
              title={`Command #${index + 1}`} // Tooltip for better UX
            >
              {/* Timestamp */}
              <span className="text-gray-400 text-xs mr-2 whitespace-nowrap">
                {new Date(entry.timestamp).toLocaleTimeString([], {
                  hour: '2-digit',
                  minute: '2-digit',
                  second: '2-digit',
                })}
              </span>
              {/* Command Content */}
              <span className="flex-1">{entry.command}</span>
            </li>
          ))
        ) : (
          // Fallback when no commands are logged
          <li className="text-center text-gray-400 py-2">
            No commands logged yet.
          </li>
        )}
      </ul>
    </div>
  );
};

export default CommandLog;
