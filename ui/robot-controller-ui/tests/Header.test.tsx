import React from 'react';
import { render, screen, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import Header from '../src/components/Header'; // Ensure this path is correct

describe('Header Component', () => {
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

  it('renders the application title', async () => {
    await act(async () => {
      render(<Header />);
    });

    const titleElement = screen.getByText(/Robot Controller/i);
    expect(titleElement).toBeInTheDocument();
  });

  it('displays the correct connection status and battery level', async () => {
    await act(async () => {
      render(<Header />);
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
    expect(screen.getByTestId('status-icon')).toHaveClass('text-green-500');

    const batteryElement = screen.getByText(/Battery:/i);
    expect(batteryElement).toBeInTheDocument();
    expect(batteryElement).toHaveTextContent('Battery:75%');
  });
});
