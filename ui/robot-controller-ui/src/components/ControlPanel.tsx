import React, { useState, useEffect } from 'react';

interface ControlPanelProps {
  onUp: () => void;
  onDown: () => void;
  onLeft: () => void;
  onRight: () => void;
  labels: { up: string, down: string, left: string, right: string };
}

const ControlPanel: React.FC<ControlPanelProps> = ({ onUp, onDown, onLeft, onRight, labels }) => {
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      let command = '';
      switch (event.key) {
        case 'w':
        case 'W':
        case 'ArrowUp':
          command = 'move-up';
          setActiveKey('up');
          break;
        case 'a':
        case 'A':
        case 'ArrowLeft':
          command = 'move-left';
          setActiveKey('left');
          break;
        case 's':
        case 'S':
        case 'ArrowDown':
          command = 'move-down';
          setActiveKey('down');
          break;
        case 'd':
        case 'D':
        case 'ArrowRight':
          command = 'move-right';
          setActiveKey('right');
          break;
        default:
          break;
      }

      if (command) {
        setTimeout(() => setActiveKey(null), 200);
      }
    };

    const handleKeyUp = () => {
      setActiveKey(null);
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  return (
    <div className="grid grid-rows-3 grid-cols-3 gap-2 w-32 h-32">
      <button
        className={`p-2 col-start-2 ${activeKey === 'up' ? 'bg-red-500' : 'bg-blue-500'} text-white`}
        onClick={onUp}
      >
        {labels.up}
      </button>
      <button
        className={`p-2 row-start-2 col-start-1 ${activeKey === 'left' ? 'bg-red-500' : 'bg-blue-500'} text-white`}
        onClick={onLeft}
      >
        {labels.left}
      </button>
      <button
        className={`p-2 row-start-2 col-start-3 ${activeKey === 'right' ? 'bg-red-500' : 'bg-blue-500'} text-white`}
        onClick={onRight}
      >
        {labels.right}
      </button>
      <button
        className={`p-2 row-start-3 col-start-2 ${activeKey === 'down' ? 'bg-red-500' : 'bg-blue-500'} text-white`}
        onClick={onDown}
      >
        {labels.down}
      </button>
    </div>
  );
};

export default ControlPanel;
