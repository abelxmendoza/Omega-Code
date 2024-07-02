import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import LineTrackerStatus from '../src/components/LineTrackerStatus';

describe('LineTrackerStatus Component', () => {
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

  it('renders the Line Tracking Status', async () => {
    await act(async () => {
      render(<LineTrackerStatus />);
    });

    expect(screen.getByText(/Line Tracking Status/i)).toBeInTheDocument();
  });

  it('updates IR sensor data from WebSocket message', async () => {
    await act(async () => {
      render(<LineTrackerStatus />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ ir01: 1, ir02: 0, ir03: 1 }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    await waitFor(() => {
      expect(screen.getByText(/IR01: 1/i)).toBeInTheDocument();
      expect(screen.getByText(/IR02: 0/i)).toBeInTheDocument();
      expect(screen.getByText(/IR03: 1/i)).toBeInTheDocument();
    });
  });
});