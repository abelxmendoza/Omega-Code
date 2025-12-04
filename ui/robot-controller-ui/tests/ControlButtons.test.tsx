import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import ControlButtons from '../src/components/control/ControlButtons';

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

describe('ControlButtons', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  it('renders ControlButtons component and triggers callbacks', () => {
    const onStart = jest.fn();
    const onStop = jest.fn();
    const onApply = jest.fn();
    const { getByText } = renderWithProviders(
      <ControlButtons onStart={onStart} onStop={onStop} onApply={onApply} />
    );
    fireEvent.click(getByText('Start'));
    expect(onStart).toHaveBeenCalled();
    fireEvent.click(getByText('Stop'));
    expect(onStop).toHaveBeenCalled();
    fireEvent.click(getByText('Apply'));
    expect(onApply).toHaveBeenCalled();
  });
});
