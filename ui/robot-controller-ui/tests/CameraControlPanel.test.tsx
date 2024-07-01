import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import CameraControlPanel from '../src/components/CameraControlPanel';
import { COMMAND } from '../src/control_definitions';
import { CommandLogProvider } from '../src/components/CommandLogContext'; // Import the provider

describe('CameraControlPanel', () => {
  const renderWithProvider = (ui) => {
    return render(
      <CommandLogProvider>
        {ui}
      </CommandLogProvider>
    );
  };

  test('renders control buttons', () => {
    const sendCommandMock = jest.fn();
    const { getByText } = renderWithProvider(<CameraControlPanel sendCommand={sendCommandMock} />);
    expect(getByText('↑')).toBeInTheDocument();
    expect(getByText('←')).toBeInTheDocument();
    expect(getByText('↓')).toBeInTheDocument();
    expect(getByText('→')).toBeInTheDocument();
  });

  test('sends correct command on button click', () => {
    const sendCommandMock = jest.fn();
    const { getByText } = renderWithProvider(<CameraControlPanel sendCommand={sendCommandMock} />);
    fireEvent.mouseDown(getByText('↑'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.CMD_SERVO_VERTICAL, 10);
  });

  test('changes button class on click', () => {
    const sendCommandMock = jest.fn();
    const { getByText } = renderWithProvider(<CameraControlPanel sendCommand={sendCommandMock} />);
    const buttonUp = getByText('↑');
    fireEvent.mouseDown(buttonUp);
    // Add assertions for button class change
    expect(buttonUp).toHaveClass('bg-gray-600');
  });
});
