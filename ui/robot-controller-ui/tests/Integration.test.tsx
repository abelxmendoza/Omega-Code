import React from 'react';
import { render, fireEvent, screen } from '@testing-library/react';
import { Provider } from 'react-redux';
import store from '../src/redux/store';
import { CommandLogProvider } from '../src/components/CommandLogContext';
import CarControlPanel from '../src/components/CarControlPanel';
import CameraControlPanel from '../src/components/CameraControlPanel';
import CommandLog from '../src/components/CommandLog';
import { COMMAND } from '../src/control_definitions';

test('Integration: CarControlPanel and CommandLog', () => {
  const mockSendCommand = jest.fn();
  const mockSendCameraCommand = jest.fn();

  render(
    <Provider store={store}>
      <CommandLogProvider>
        <CarControlPanel sendCommand={mockSendCommand} />
        <CameraControlPanel sendCommand={mockSendCameraCommand} />
        <CommandLog />
      </CommandLogProvider>
    </Provider>
  );

  // Simulate car control key presses
  fireEvent.keyDown(window, { key: 'w' });
  fireEvent.keyUp(window, { key: 'w' });
  fireEvent.keyDown(window, { key: 's' });
  fireEvent.keyUp(window, { key: 's' });

  // Simulate camera control key presses
  fireEvent.keyDown(window, { key: 'ArrowUp' });
  fireEvent.keyUp(window, { key: 'ArrowUp' });
  fireEvent.keyDown(window, { key: 'ArrowDown' });
  fireEvent.keyUp(window, { key: 'ArrowDown' });

  // Check if the commands are sent
  expect(mockSendCommand).toHaveBeenCalledWith(COMMAND.MOVE_UP);
  expect(mockSendCommand).toHaveBeenCalledWith(COMMAND.MOVE_DOWN);
  expect(mockSendCameraCommand).toHaveBeenCalledWith(COMMAND.CMD_SERVO_VERTICAL, 10);
  expect(mockSendCameraCommand).toHaveBeenCalledWith(COMMAND.CMD_SERVO_VERTICAL, -10);

  // Check if the commands are in the CommandLog
  // Assuming CommandLog component displays commands sent
  expect(screen.getByText('move-up')).toBeInTheDocument();
  expect(screen.getByText('move-down')).toBeInTheDocument();
  // Add more checks if necessary for other commands
});
