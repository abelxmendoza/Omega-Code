import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import Status from './Status';

interface HeaderProps {
  isConnected: boolean;
  batteryLevel: number;
}

const Header: React.FC<HeaderProps> = ({ isConnected, batteryLevel }) => {
  return (
    <div className="flex justify-between items-center bg-gray-800 text-white p-4">
      <div className="text-lg font-bold">Robot Controller</div>
      <Status status={isConnected ? 'Connected' : 'Disconnected'} battery={batteryLevel} />
    </div>
  );
};

export default Header;
