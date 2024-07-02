import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import Status from '../src/components/Status';

describe('Status', () => {
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

  it('displays the correct status icon and text when connected', async () => {
    await act(async () => {
      render(<Status />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ battery: 75 }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    const statusElement = screen.getByText(/Status:/i).parentElement;
    expect(statusElement).toBeInTheDocument();
    expect(screen.getByText(/Status:/i)).toBeInTheDocument();
    expect(screen.getByText(/Battery:/i)).toBeInTheDocument();
    expect(screen.getByText(/75%/i)).toBeInTheDocument();
    expect(screen.getByTestId('status-icon')).toHaveClass('text-green-500');
  });

  it('displays the correct status icon and text when disconnected', () => {
    render(<Status status="Disconnected" battery={15} />);

    const statusElement = screen.getByText(/Status:/i).parentElement;
    expect(statusElement).toBeInTheDocument();
    expect(screen.getByText(/Status:/i)).toBeInTheDocument();
    expect(screen.getByText(/Battery:/i)).toBeInTheDocument();
    expect(screen.getByText(/15%/i)).toBeInTheDocument();
    expect(screen.getByTestId('status-icon')).toHaveClass('text-red-500');
  });

  it('displays the correct battery bar color based on battery level', () => {
    const { rerender } = render(<Status status="Connected" battery={75} />);
    const batteryBar = screen.getByText(/75%/i).previousElementSibling.firstChild;
    expect(batteryBar).toHaveClass('bg-blue-500');

    rerender(<Status status="Connected" battery={15} />);
    const batteryBarUpdated = screen.getByText(/15%/i).previousElementSibling.firstChild;
    expect(batteryBarUpdated).toHaveClass('bg-red-500');
  });
});
