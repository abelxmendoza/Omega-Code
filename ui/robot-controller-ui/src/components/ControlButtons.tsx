import React from 'react';

interface ControlButtonsProps {
  onButtonClick?: (button: string) => void;
}

const ControlButtons: React.FC<ControlButtonsProps> = ({ 
  onButtonClick 
}) => {
  return (
    <div>
      <h3>Control Buttons</h3>
      <button onClick={() => onButtonClick?.('button1')}>Button 1</button>
      <button onClick={() => onButtonClick?.('button2')}>Button 2</button>
    </div>
  );
};

export default ControlButtons;
