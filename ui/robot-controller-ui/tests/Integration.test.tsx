import React from 'react';
import { render, fireEvent, screen, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import CarControlPanel from '../src/components/control/CarControlPanel';
import CameraControlPanel from '../src/components/control/CameraControlPanel';
import CommandLog from '../src/components/CommandLog';

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

test('Integration: CarControlPanel and CommandLog', async () => {
  await act(async () => {
    renderWithProviders(
      <>
        <CarControlPanel />
        <CameraControlPanel />
        <CommandLog />
      </>
    );
  });

  await act(async () => {
    await new Promise(resolve => setTimeout(resolve, 100));
    
    // Simulate car control key presses
    fireEvent.keyDown(window, { key: 'w' });
    fireEvent.keyUp(window, { key: 'w' });
    fireEvent.keyDown(window, { key: 's' });
    fireEvent.keyUp(window, { key: 's' });

    // Simulate camera control key presses
    fireEvent.keyDown(window, { key: 'ArrowUp' });
    fireEvent.keyUp(window, { key: 'ArrowUp' });
    fireEvent.keyDown(window, { key: 'ArrowDown' });
    fireEvent.keyUp(window, { key: 'ArrowDown' });
  });

  // Components should render and handle events without crashing
  expect(document.body).toBeInTheDocument();
  expect(screen.getByText(/Command Log/i)).toBeInTheDocument();
});

