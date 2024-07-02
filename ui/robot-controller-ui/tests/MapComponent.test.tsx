import React from 'react';
import { render, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import MapComponent from '../src/components/MapComponent';

const mockWebSocket = {
  send: jest.fn(),
  close: jest.fn(),
  addEventListener: jest.fn(),
  removeEventListener: jest.fn(),
};

global.WebSocket = jest.fn(() => mockWebSocket);

describe('MapComponent', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders map container', () => {
    const { container } = render(<MapComponent />);
    const mapElement = container.querySelector('#map');
    expect(mapElement).toBeInTheDocument();
  });

  test('establishes WebSocket connection', async () => {
    await act(async () => {
      render(<MapComponent />);
    });

    await waitFor(() => {
      expect(global.WebSocket).toHaveBeenCalledTimes(1);
      expect(mockWebSocket.addEventListener).toHaveBeenCalledWith('open', expect.any(Function));
    });
  });

  test('updates marker position on WebSocket message', async () => {
    const { container } = render(<MapComponent />);

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ type: 'location', lat: 51.515, lng: -0.1 }),
    });

    await act(async () => {
      const messageHandler = mockWebSocket.addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    await waitFor(() => {
      const marker = container.querySelector('.leaflet-marker-icon');
      expect(marker).toBeInTheDocument();
      expect(marker.style.transform).toContain('translate3d');
    });
  });
});
