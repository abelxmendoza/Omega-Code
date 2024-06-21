// File: /Omega-Code/ui/robot-controller-ui/src/components/Header.tsx
import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import Status from './Status';

const Header: React.FC<{ isConnected: boolean; batteryLevel: number }> = ({ isConnected, batteryLevel }) => {
  return (
    <div className="sticky top-0 flex justify-between items-center bg-gray-800 text-white p-4">
      <div className="text-lg font-bold">Robot Controller</div>
      <Status status={isConnected ? 'Connected' : 'Disconnected'} battery={batteryLevel} />
    </div>
  );
};

export default Header;
