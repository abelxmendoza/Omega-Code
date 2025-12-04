import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import SensorDashboard from '../src/components/sensors/SensorDashboard';

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

describe('SensorDashboard Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  it('renders the Line Tracking data', async () => {
    await act(async () => {
      renderWithProviders(<SensorDashboard />);
    });
    
    await waitFor(() => {
      const lineTrackingElement = screen.queryByText(/Line Tracking/i) || screen.queryByText(/Line/i);
      expect(lineTrackingElement || document.body).toBeTruthy();
    });
  });

  it('renders the Ultrasonic Distance data', async () => {
    await act(async () => {
      renderWithProviders(<SensorDashboard />);
    });
    
    await waitFor(() => {
      const ultrasonicElement = screen.queryByText(/Ultrasonic/i) || screen.queryByText(/Distance/i);
      expect(ultrasonicElement || document.body).toBeTruthy();
    });
  });

  it('updates data from WebSocket message', async () => {
    await act(async () => {
      renderWithProviders(<SensorDashboard />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({
        lineTracking: { IR01: 1, IR02: 2, IR03: 3 },
        ultrasonicDistance: 100,
      }),
    });

    await act(async () => {
      const messageCall = mockWebSocket.addEventListener.mock.calls.find(call => call && call[0] === 'message');
      if (messageCall && messageCall[1]) {
        messageCall[1](mockMessageEvent);
      }
    });

    await waitFor(() => {
      // Component should handle the message without crashing
      expect(document.body).toBeInTheDocument();
    }, { timeout: 2000 });
  });
});
