// src/components/ControlPanel.tsx

/*
This component represents a control panel for the robot that can be operated using either WASD keys or arrow keys.
It allows the user to control the robot's movements (up, down, left, right) and provides visual feedback on the active control.
*/

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
    // Function to handle keydown events
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (controlType) {
        case 'wasd':
          switch (event.key) {
            case 'w':
            case 'W':
              setActiveKey('up');
              onUp();
              break;
            case 'a':
            case 'A':
              setActiveKey('left');
              onLeft();
              break;
            case 's':
            case 'S':
              setActiveKey('down');
              onDown();
              break;
            case 'd':
            case 'D':
              setActiveKey('right');
              onRight();
              break;
            default:
              break;
          }
          break;
        case 'arrows':
          switch (event.key) {
            case 'ArrowUp':
              setActiveKey('up');
              onUp();
              break;
            case 'ArrowLeft':
              setActiveKey('left');
              onLeft();
              break;
            case 'ArrowDown':
              setActiveKey('down');
              onDown();
              break;
            case 'ArrowRight':
              setActiveKey('right');
              onRight();
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    };

    // Function to handle keyup events
    const handleKeyUp = (event: KeyboardEvent) => {
      switch (controlType) {
        case 'wasd':
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
          break;
        case 'arrows':
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
          break;
        default:
          break;
      }
    };

    // Add event listeners for keydown and keyup
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    // Cleanup event listeners on component unmount
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [controlType, activeKey, onUp, onDown, onLeft, onRight]);

  // Function to get the button class based on the active key
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
