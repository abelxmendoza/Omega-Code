import React from 'react';

const ControlPanel: React.FC = () => {
  return (
    <div className="grid grid-cols-3 gap-4 w-32">
      <button className="bg-blue-500 text-white p-2">Up</button>
      <button className="bg-blue-500 text-white p-2">Left</button>
      <button className="bg-blue-500 text-white p-2">Right</button>
      <button className="bg-blue-500 text-white p-2">Down</button>
    </div>
  );
};

export default ControlPanel;
