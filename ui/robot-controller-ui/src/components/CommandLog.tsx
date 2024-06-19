import React from 'react';

const CommandLog: React.FC = () => {
  const commands = ["Command 1", "Command 2", "Command 3"];
  return (
    <div className="p-4 bg-gray-100">
      <h2 className="text-lg">Command Log</h2>
      <ul>
        {commands.map((command, index) => (
          <li key={index}>{command}</li>
        ))}
      </ul>
    </div>
  );
};

export default CommandLog;
