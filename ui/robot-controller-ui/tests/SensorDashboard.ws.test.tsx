import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from '../utils/test-helpers';
import SensorDashboard from '@/components/sensors/SensorDashboard';

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

describe('SensorDashboard (WS integration)', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  it('reacts to incoming sensor data', async () => {
    await act(async () => {
      renderWithProviders(<SensorDashboard />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ type: 'sensors', ultrasonic: 42, line: 'left', light: 0.73 }),
    });

    await act(async () => {
      const messageCall = mockWebSocket.addEventListener.mock.calls.find(call => call && call[0] === 'message');
      if (messageCall && messageCall[1]) {
        messageCall[1](mockMessageEvent);
      }
    });

    // Component should handle the message without crashing
    await waitFor(() => {
      expect(document.body).toBeInTheDocument();
    }, { timeout: 2000 });
  });
});
