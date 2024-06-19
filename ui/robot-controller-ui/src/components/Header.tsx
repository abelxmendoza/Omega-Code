import React from 'react';
import Status from './Status';

const Header: React.FC = () => {
  return (
    <header className="flex items-center justify-between p-4 bg-gray-800 text-white">
      <h1 className="text-2xl">Robot Controller</h1>
      <Status status="Connected" battery={80} />
    </header>
  );
};

export default Header;
