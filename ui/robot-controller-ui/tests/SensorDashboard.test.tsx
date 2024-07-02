import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import SensorDashboard from '../src/components/SensorDashboard';

beforeAll(() => {
  global.WebSocket = jest.fn(() => ({
    addEventListener: jest.fn(),
    close: jest.fn(),
  }));
});

describe('SensorDashboard Component', () => {
  beforeEach(() => {
    global.WebSocket.mockClear();
  });

  it('renders the Line Tracking data', () => {
    render(<SensorDashboard />);
    const lineTrackingElement = screen.getByText(/Line Tracking:/i);
    expect(lineTrackingElement).toBeInTheDocument();
  });

  it('renders the Ultrasonic Distance data', () => {
    render(<SensorDashboard />);
    const ultrasonicDistanceElement = screen.getByText(/Ultrasonic Distance:/i);
    expect(ultrasonicDistanceElement).toBeInTheDocument();
  });

  it('updates data from WebSocket message', async () => {
    await act(async () => {
      render(<SensorDashboard />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({
        lineTracking: { IR01: 1, IR02: 2, IR03: 3 },
        ultrasonicDistance: 100,
      }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    await waitFor(() => {
      expect(screen.getByText(/IR01: 1/i)).toBeInTheDocument();
      expect(screen.getByText(/IR02: 2/i)).toBeInTheDocument();
      expect(screen.getByText(/IR03: 3/i)).toBeInTheDocument();
      expect(screen.getByText(/Distance: 100 cm/i)).toBeInTheDocument();
    });
  });
});
