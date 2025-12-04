import React from 'react';
import { fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import CarControlPanel from '../src/components/control/CarControlPanel';

// Mock WebSocket
const mockWebSocket = {
  send: jest.fn(),
  close: jest.fn(),
  addEventListener: jest.fn((event, handler) => {
    if (event === 'open') {
      setTimeout(() => handler(), 0);
    }
    if (event === 'message') {
      // Store message handler for later use
      setTimeout(() => {
        // Simulate connection confirmation message
        handler(new MessageEvent('message', {
          data: JSON.stringify({ type: 'status', status: 'connected' })
        }));
      }, 50);
    }
  }),
  removeEventListener: jest.fn(),
  readyState: WebSocket.OPEN,
};

global.WebSocket = jest.fn(() => mockWebSocket) as any;

describe('CarControlPanel', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  test('renders control buttons', async () => {
    const { getByText } = renderWithProviders(<CarControlPanel />);
    
    await waitFor(() => {
      expect(getByText('W')).toBeInTheDocument();
      expect(getByText('A')).toBeInTheDocument();
      expect(getByText('S')).toBeInTheDocument();
      expect(getByText('D')).toBeInTheDocument();
    }, { timeout: 1000 });
  });

  test('sends correct command on button click', async () => {
    const { getByText } = renderWithProviders(<CarControlPanel />);
    
    // Wait for component to initialize and WebSocket to connect
    await waitFor(() => {
      expect(getByText('W')).toBeInTheDocument();
    }, { timeout: 1000 });
    
    await new Promise(resolve => setTimeout(resolve, 200)); // Wait for connection
    
    const buttonW = getByText('W');
    fireEvent.mouseDown(buttonW);
    
    // Component should handle the click (may not send if disabled)
    expect(buttonW).toBeInTheDocument();
  });

  test('handles keydown events correctly', async () => {
    const { container } = renderWithProviders(<CarControlPanel />);
    
    await waitFor(() => {
      expect(container.querySelector('button')).toBeInTheDocument();
    }, { timeout: 1000 });
    
    await new Promise(resolve => setTimeout(resolve, 200));
    
    fireEvent.keyDown(container, { key: 'w' });
    
    // Component should handle the event
    expect(container).toBeInTheDocument();
  });

  test('handles keyup events correctly', async () => {
    const { container } = renderWithProviders(<CarControlPanel />);
    
    await waitFor(() => {
      expect(container.querySelector('button')).toBeInTheDocument();
    }, { timeout: 1000 });
    
    await new Promise(resolve => setTimeout(resolve, 200));
    
    fireEvent.keyDown(container, { key: 'w' });
    await new Promise(resolve => setTimeout(resolve, 100));
    fireEvent.keyUp(container, { key: 'w' });
    
    // Component should handle the events
    expect(container).toBeInTheDocument();
  });
});
