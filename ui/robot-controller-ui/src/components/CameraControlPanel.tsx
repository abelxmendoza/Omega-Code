import React from 'react';

interface CameraControlPanelProps {
  onCommand?: (command: string) => void;
}

const CameraControlPanel: React.FC<CameraControlPanelProps> = ({ 
  onCommand 
}) => {
  return (
    <div>
      <h3>Camera Control Panel</h3>
      <button onClick={() => onCommand?.('up')}>Up</button>
      <button onClick={() => onCommand?.('down')}>Down</button>
      <button onClick={() => onCommand?.('left')}>Left</button>
      <button onClick={() => onCommand?.('right')}>Right</button>
    </div>
  );
};

export default CameraControlPanel;
