import React from 'react';

interface CarControlPanelProps {
  onCommand?: (command: string) => void;
}

const CarControlPanel: React.FC<CarControlPanelProps> = ({ 
  onCommand 
}) => {
  return (
    <div>
      <h3>Car Control Panel</h3>
      <button onClick={() => onCommand?.('forward')}>Forward</button>
      <button onClick={() => onCommand?.('backward')}>Backward</button>
      <button onClick={() => onCommand?.('left')}>Left</button>
      <button onClick={() => onCommand?.('right')}>Right</button>
    </div>
  );
};

export default CarControlPanel;
