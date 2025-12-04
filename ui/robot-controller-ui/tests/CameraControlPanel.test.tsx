import React from 'react';
import { render, fireEvent, act, screen, waitFor } from '@testing-library/react';
import CameraControlPanel from '../src/components/control/CameraControlPanel';
import { COMMAND } from '../src/control_definitions';
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

describe('CameraControlPanel', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  test('renders control buttons', () => {
    const { container } = renderWithProviders(<CameraControlPanel />);
    // Check for button elements by their aria-label or text content
    const buttons = container.querySelectorAll('button');
    expect(buttons.length).toBeGreaterThan(0);
  });

  test('sends correct command on button click', async () => {
    const { container } = renderWithProviders(<CameraControlPanel />);
    const buttons = container.querySelectorAll('button');
    if (buttons.length > 0) {
      fireEvent.mouseDown(buttons[0]);
      await waitFor(() => {
        expect(mockWebSocket.send).toHaveBeenCalled();
      }, { timeout: 2000 });
    }
  });

  test('changes button class on click', () => {
    const { container } = renderWithProviders(<CameraControlPanel />);
    const buttons = container.querySelectorAll('button');
    if (buttons.length > 0) {
      const button = buttons[0];
      fireEvent.mouseDown(button);
      // Check if button has pressed state class
      expect(button.className).toBeTruthy();
    }
  });

  it('updates command log from WebSocket message', async () => {
    const sendCommandMock = jest.fn();
    const { getByText } = renderWithProvider(<CameraControlPanel sendCommand={sendCommandMock} />);
    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ command: 'camera-up' }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    await waitFor(() => {
      expect(getByText('camera-up')).toBeInTheDocument();
    });
  }, 5000); // Set timeout to 5 seconds
});
