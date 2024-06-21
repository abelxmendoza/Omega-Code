import React, { useState, useEffect } from 'react';

interface ControlPanelProps {
  onUp: () => void;
  onDown: () => void;
  onLeft: () => void;
  onRight: () => void;
  labels: { up: string, down: string, left: string, right: string };
  controlType: 'wasd' | 'arrows';
}

const ControlPanel: React.FC<ControlPanelProps> = ({ onUp, onDown, onLeft, onRight, labels, controlType }) => {
  const [activeKey, setActiveKey] = useState<string | null>(null);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (controlType === 'wasd') {
        switch (event.key) {
          case 'w':
          case 'W':
            setActiveKey('up');
            break;
          case 'a':
          case 'A':
            setActiveKey('left');
            break;
          case 's':
          case 'S':
            setActiveKey('down');
            break;
          case 'd':
          case 'D':
            setActiveKey('right');
            break;
          default:
            break;
        }
      } else if (controlType === 'arrows') {
        switch (event.key) {
          case 'ArrowUp':
            setActiveKey('up');
            break;
          case 'ArrowLeft':
            setActiveKey('left');
            break;
          case 'ArrowDown':
            setActiveKey('down');
            break;
          case 'ArrowRight':
            setActiveKey('right');
            break;
          default:
            break;
        }
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      if (controlType === 'wasd') {
        switch (event.key) {
          case 'w':
          case 'W':
            if (activeKey === 'up') setActiveKey(null);
            break;
          case 'a':
          case 'A':
            if (activeKey === 'left') setActiveKey(null);
            break;
          case 's':
          case 'S':
            if (activeKey === 'down') setActiveKey(null);
            break;
          case 'd':
          case 'D':
            if (activeKey === 'right') setActiveKey(null);
            break;
          default:
            break;
        }
      } else if (controlType === 'arrows') {
        switch (event.key) {
          case 'ArrowUp':
            if (activeKey === 'up') setActiveKey(null);
            break;
          case 'ArrowLeft':
            if (activeKey === 'left') setActiveKey(null);
            break;
          case 'ArrowDown':
            if (activeKey === 'down') setActiveKey(null);
            break;
          case 'ArrowRight':
            if (activeKey === 'right') setActiveKey(null);
            break;
          default:
            break;
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [controlType, activeKey]);

  const getButtonClass = (key: string) => {
    return activeKey === key ? 'bg-red-500' : 'bg-blue-500';
  };

  return (
    <div className="grid grid-rows-3 grid-cols-3 gap-2 w-32 h-32">
      <button
        className={`p-2 col-start-2 rounded ${getButtonClass('up')} text-white`}
        onClick={onUp}
      >
        {labels.up}
      </button>
      <button
        className={`p-2 row-start-2 col-start-1 rounded ${getButtonClass('left')} text-white`}
        onClick={onLeft}
      >
        {labels.left}
      </button>
      <button
        className={`p-2 row-start-2 col-start-3 rounded ${getButtonClass('right')} text-white`}
        onClick={onRight}
      >
        {labels.right}
      </button>
      <button
        className={`p-2 row-start-3 col-start-2 rounded ${getButtonClass('down')} text-white`}
        onClick={onDown}
      >
        {labels.down}
      </button>
    </div>
  );
};

export default ControlPanel;
