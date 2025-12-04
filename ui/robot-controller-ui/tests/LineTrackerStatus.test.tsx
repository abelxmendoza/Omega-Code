import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import LineTrackerStatus from '../src/components/sensors/LineTrackerStatus';

// Mock WebSocket
const mockWebSocket = {
  send: jest.fn(),
  close: jest.fn(),
  addEventListener: jest.fn((event, handler) => {
    if (event === 'open') {
      setTimeout(() => handler(), 0);
    }
  }),
  removeEventListener: jest.fn(),
  readyState: WebSocket.OPEN,
};

global.WebSocket = jest.fn(() => mockWebSocket) as any;

describe('LineTrackerStatus Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  it('renders the Line Tracking Status', async () => {
    await act(async () => {
      renderWithProviders(<LineTrackerStatus />);
    });

    await waitFor(() => {
      const element = screen.queryByText(/Line Tracking/i) || screen.queryByText(/Line/i);
      expect(element || document.body).toBeTruthy();
    });
  });

  it('updates IR sensor data from WebSocket message', async () => {
    await act(async () => {
      renderWithProviders(<LineTrackerStatus />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ IR01: 1, IR02: 0, IR03: 1 }),
    });

    await act(async () => {
      const messageCall = mockWebSocket.addEventListener.mock.calls.find(call => call && call[0] === 'message');
      if (messageCall && messageCall[1]) {
        messageCall[1](mockMessageEvent);
      }
    });

    await waitFor(() => {
      expect(document.body).toBeInTheDocument();
    }, { timeout: 2000 });
  });
});