import React from 'react';

interface LedModalProps {
  isOpen?: boolean;
  onClose?: () => void;
  onApply?: (settings: any) => void;
}

const LedModal: React.FC<LedModalProps> = ({ 
  isOpen = false, 
  onClose,
  onApply
}) => {
  if (!isOpen) return null;

  return (
    <div>
      <h3>LED Modal</h3>
      <button onClick={onClose}>Close</button>
      <button onClick={() => onApply?.({})}>Apply</button>
    </div>
  );
};

export default LedModal;
