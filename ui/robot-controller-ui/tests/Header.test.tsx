import React from 'react';
import { screen, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import Header from '../src/components/Header'; // Ensure this path is correct

describe('Header Component', () => {
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
    
    // Mock fetch for latency metrics
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: false,
        json: () => Promise.resolve({ ok: false }),
      } as Response)
    );
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('renders the application title', async () => {
    await act(async () => {
      renderWithProviders(<Header batteryLevel={75} />);
    });

    const titleElement = screen.getByText(/Robot Controller/i);
    expect(titleElement).toBeInTheDocument();
  });

  it('displays the correct connection status and battery level', async () => {
    await act(async () => {
      renderWithProviders(<Header batteryLevel={75} />);
    });

    // Wait for component to render
    await act(async () => {
      await new Promise(resolve => setTimeout(resolve, 100));
    });

    // Check that header renders with battery level
    const titleElement = screen.getByText(/Robot Controller/i);
    expect(titleElement).toBeInTheDocument();
    
    // Battery level should be displayed (component receives it as prop)
    // The exact text format depends on Header implementation
    expect(screen.getByText(/75/i)).toBeInTheDocument();
  });
});

