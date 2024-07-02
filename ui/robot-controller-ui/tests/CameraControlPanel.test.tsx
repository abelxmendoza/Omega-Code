import React from 'react';
import { render, fireEvent, act, screen, waitFor } from '@testing-library/react';
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

  beforeEach(() => {
    // Mock WebSocket
    global.WebSocket = jest.fn(() => ({
      send: jest.fn(),
      close: jest.fn(),
      addEventListener: jest.fn((event, handler) => {
        if (event === 'open') {
          handler();
        }
      }),
      removeEventListener: jest.fn(),
    }));
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

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
    expect(buttonUp).toHaveClass('bg-gray-600');
  });

  it('updates command log from WebSocket message', async () => {
    const sendCommandMock = jest.fn();
    const { getByText } = renderWithProvider(<CameraControlPanel sendCommand={sendCommandMock} />);
    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ command: 'camera-up' }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    await waitFor(() => {
      expect(getByText('camera-up')).toBeInTheDocument();
    });
  }, 5000); // Set timeout to 5 seconds
});
