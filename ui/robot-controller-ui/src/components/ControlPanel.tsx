import React from 'react';
import CarControlPanel from './CarControlPanel';
import CameraControlPanel from './CameraControlPanel';
import { useDispatch } from 'react-redux';
import { executeCommand } from '../redux/reducers/controlPanelReducer';
import { useCommandLog } from './CommandLogContext';

interface ControlPanelProps {
  controlType: 'wasd' | 'arrows';
}

const ControlPanel: React.FC<ControlPanelProps> = ({ controlType }) => {
  const dispatch = useDispatch();
  const { addCommand } = useCommandLog();

  const sendCarCommand = (command: string) => {
    dispatch(executeCommand(command));
    addCommand(command);
  };

  const sendCameraCommand = (command: string, angle: number) => {
    dispatch(executeCommand(command, angle));
    addCommand(`${command} ${angle}`);
  };

  return (
    <div className="control-panel-container">
      {controlType === 'wasd' ? (
        <CarControlPanel sendCommand={sendCarCommand} />
      ) : (
        <CameraControlPanel sendCommand={sendCameraCommand} />
      )}
    </div>
  );
};

export default ControlPanel;
