import React from 'react';

interface ControlPanelProps {
  onUp: () => void;
  onDown: () => void;
  onLeft: () => void;
  onRight: () => void;
}

const ControlPanel: React.FC<ControlPanelProps> = ({ onUp, onDown, onLeft, onRight }) => {
  return (
    <div className="grid grid-rows-3 grid-cols-3 gap-2 w-32 h-32">
      <button className="bg-blue-500 text-white p-2 col-start-2" onClick={onUp}>Up</button>
      <button className="bg-blue-500 text-white p-2 row-start-2 col-start-1" onClick={onLeft}>Left</button>
      <button className="bg-blue-500 text-white p-2 row-start-2 col-start-3" onClick={onRight}>Right</button>
      <button className="bg-blue-500 text-white p-2 row-start-3 col-start-2" onClick={onDown}>Down</button>
    </div>
  );
};

export default ControlPanel;
