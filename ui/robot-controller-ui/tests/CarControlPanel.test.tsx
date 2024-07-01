import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import CarControlPanel from '../src/components/CarControlPanel';
import { COMMAND } from '../src/control_definitions'; // Import the COMMAND constant
import { CommandLogProvider } from '../src/components/CommandLogContext'; // Import the provider

describe('CarControlPanel', () => {
  let sendCommandMock;

  beforeEach(() => {
    sendCommandMock = jest.fn();
  });

  const renderWithProvider = (ui) => {
    return render(
      <CommandLogProvider>
        {ui}
      </CommandLogProvider>
    );
  };

  test('renders control buttons', () => {
    const { getByText } = renderWithProvider(<CarControlPanel sendCommand={sendCommandMock} />);
    expect(getByText('W')).toBeInTheDocument();
    expect(getByText('A')).toBeInTheDocument();
    expect(getByText('S')).toBeInTheDocument();
    expect(getByText('D')).toBeInTheDocument();
  });

  test('sends correct command on button click', () => {
    const { getByText } = renderWithProvider(<CarControlPanel sendCommand={sendCommandMock} />);
    fireEvent.mouseDown(getByText('W'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.MOVE_UP);
  });

  test('changes button color on click', () => {
    const { getByText } = renderWithProvider(<CarControlPanel sendCommand={sendCommandMock} />);
    const buttonW = getByText('W');
    fireEvent.mouseDown(buttonW);
    expect(buttonW).toHaveClass('bg-gray-600');
  });

  test('handles keydown events correctly', () => {
    const { container } = renderWithProvider(<CarControlPanel sendCommand={sendCommandMock} />);
    fireEvent.keyDown(container, { key: 'w' });
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.MOVE_UP);
  });

  test('handles keyup events correctly', () => {
    const { container } = renderWithProvider(<CarControlPanel sendCommand={sendCommandMock} />);
    fireEvent.keyDown(container, { key: 'w' });
    fireEvent.keyUp(container, { key: 'w' });
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.STOP);
  });
});
