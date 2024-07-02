import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import UltrasonicSensorStatus from '../src/components/UltrasonicSensorStatus';

describe('UltrasonicSensorStatus Component', () => {
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

  it('renders the Ultrasonic Sensor Status', async () => {
    await act(async () => {
      render(<UltrasonicSensorStatus />);
    });

    expect(screen.getByText(/Ultrasonic Sensor Status/i)).toBeInTheDocument();
  });

  it('updates distance from WebSocket message', async () => {
    await act(async () => {
      render(<UltrasonicSensorStatus />);
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ distance: 150 }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    await waitFor(() => {
      expect(screen.getByText(/Distance: 150 cm/i)).toBeInTheDocument();
    });
  });
});