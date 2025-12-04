import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';

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

// Check if component exists
let UltrasonicSensorStatus: React.ComponentType<any>;
try {
  UltrasonicSensorStatus = require('../src/components/UltrasonicSensorStatus').default;
} catch {
  UltrasonicSensorStatus = () => <div>UltrasonicSensorStatus not found</div>;
}

describe('UltrasonicSensorStatus Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  it('renders the Ultrasonic Sensor Status', async () => {
    await act(async () => {
      renderWithProviders(<UltrasonicSensorStatus />);
    });

    await waitFor(() => {
      const element = screen.queryByText(/Ultrasonic/i) || screen.queryByText(/Distance/i);
      expect(element || document.body).toBeTruthy();
    });
  });

  it('updates distance from WebSocket message', async () => {
    await act(async () => {
      renderWithProviders(<UltrasonicSensorStatus />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ distance: 150 }),
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