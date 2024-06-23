import React, { useState, useEffect } from 'react';
import { useDispatch } from 'react-redux';
import { executeCommand } from '../redux/reducers/controlPanelReducer';

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
  const dispatch = useDispatch();

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      console.log(`Key pressed in ControlPanel (${controlType}): ${event.key}`);
      let command = '';
      switch (controlType) {
        case 'wasd':
          switch (event.key) {
            case 'w':
            case 'W':
              setActiveKey('up');
              onUp();
              command = 'move-up';
              break;
            case 'a':
            case 'A':
              setActiveKey('left');
              onLeft();
              command = 'move-left';
              break;
            case 's':
            case 'S':
              setActiveKey('down');
              onDown();
              command = 'move-down';
              break;
            case 'd':
            case 'D':
              setActiveKey('right');
              onRight();
              command = 'move-right';
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
              command = 'move-up';
              break;
            case 'ArrowLeft':
              setActiveKey('left');
              onLeft();
              command = 'move-left';
              break;
            case 'ArrowDown':
              setActiveKey('down');
              onDown();
              command = 'move-down';
              break;
            case 'ArrowRight':
              setActiveKey('right');
              onRight();
              command = 'move-right';
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
      if (command) {
        dispatch(executeCommand(command));
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      console.log(`Key released in ControlPanel (${controlType}): ${event.key}`);
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

    console.log('Adding event listeners in ControlPanel');
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      console.log('Removing event listeners in ControlPanel');
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [controlType, activeKey, onUp, onDown, onLeft, onRight, dispatch]);

  const getButtonClass = (key: string) => {
    return activeKey === key ? 'bg-red-500' : 'bg-blue-500';
  };

  return (
    <div className="grid grid-rows-3 grid-cols-3 gap-2 w-32 h-32">
      <button
        className={`p-2 col-start-2 rounded ${getButtonClass('up')} text-white`}
        onClick={() => {
          onUp();
          dispatch(executeCommand('move-up'));
        }}
      >
        {labels.up}
      </button>
      <button
        className={`p-2 row-start-2 col-start-1 rounded ${getButtonClass('left')} text-white`}
        onClick={() => {
          onLeft();
          dispatch(executeCommand('move-left'));
        }}
      >
        {labels.left}
      </button>
      <button
        className={`p-2 row-start-2 col-start-3 rounded ${getButtonClass('right')} text-white`}
        onClick={() => {
          onRight();
          dispatch(executeCommand('move-right'));
        }}
      >
        {labels.right}
      </button>
      <button
        className={`p-2 row-start-3 col-start-2 rounded ${getButtonClass('down')} text-white`}
        onClick={() => {
          onDown();
          dispatch(executeCommand('move-down'));
        }}
      >
        {labels.down}
      </button>
    </div>
  );
};

export default ControlPanel;
