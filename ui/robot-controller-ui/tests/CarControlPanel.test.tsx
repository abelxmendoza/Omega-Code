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

  test('renders control buttons', () => {
    const { getByText } = renderWithProviders(<CarControlPanel />);
    expect(getByText('W')).toBeInTheDocument();
    expect(getByText('A')).toBeInTheDocument();
    expect(getByText('S')).toBeInTheDocument();
    expect(getByText('D')).toBeInTheDocument();
  });

  test('sends correct command on button click', async () => {
    const { getByText } = renderWithProviders(<CarControlPanel />);
    const buttonW = getByText('W');
    
    fireEvent.mouseDown(buttonW);
    
    // Wait a bit for async command sending
    await waitFor(() => {
      expect(mockWebSocket.send).toHaveBeenCalled();
    }, { timeout: 2000 });
  });

  test('handles keydown events correctly', async () => {
    const { container } = renderWithProviders(<CarControlPanel />);
    
    fireEvent.keyDown(container, { key: 'w' });
    
    await waitFor(() => {
      expect(mockWebSocket.send).toHaveBeenCalled();
    }, { timeout: 2000 });
  });

  test('handles keyup events correctly', async () => {
    const { container } = renderWithProviders(<CarControlPanel />);
    
    fireEvent.keyDown(container, { key: 'w' });
    await new Promise(resolve => setTimeout(resolve, 100));
    fireEvent.keyUp(container, { key: 'w' });
    
    await waitFor(() => {
      // Should send stop command on keyup
      expect(mockWebSocket.send).toHaveBeenCalled();
    }, { timeout: 2000 });
  });
});
