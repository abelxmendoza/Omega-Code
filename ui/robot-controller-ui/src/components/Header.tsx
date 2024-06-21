import React from 'react';
import Status from './Status';

const Header: React.FC<{ isConnected: boolean; batteryLevel: number }> = ({ isConnected, batteryLevel }) => {
  return (
    <div className="sticky top-0 flex justify-between items-center bg-gray-800 text-white p-4 z-50">
      <div className="text-lg font-bold">Robot Controller</div>
      <Status status={isConnected ? 'Connected' : 'Disconnected'} battery={batteryLevel} />
    </div>
  );
};

export default Header;
