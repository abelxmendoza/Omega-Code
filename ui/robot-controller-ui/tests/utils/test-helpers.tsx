/**
 * Test Utilities and Helpers
 * 
 * Common utilities for testing React components and hooks
 */

import React from 'react';
import { render, RenderOptions } from '@testing-library/react';
import { Provider } from 'react-redux';
import { configureStore } from '@reduxjs/toolkit';
import { CommandProvider } from '@/context/CommandContext';
import { CapabilityProvider } from '@/context/CapabilityContext';

// Mock store for testing
export const createMockStore = (initialState = {}) => {
  // Create a simple reducer that returns state as-is
  const rootReducer = (state = initialState) => state;
  
  return configureStore({
    reducer: rootReducer,
    preloadedState: initialState,
  });
};

// Custom render function with providers
export const renderWithProviders = (
  ui: React.ReactElement,
  {
    preloadedState = {},
    store = createMockStore(preloadedState),
    ...renderOptions
  }: RenderOptions & { preloadedState?: any; store?: any } = {}
) => {
  const Wrapper = ({ children }: { children: React.ReactNode }) => {
    return (
      <Provider store={store}>
        <CapabilityProvider>
          <CommandProvider>
            {children}
          </CommandProvider>
        </CapabilityProvider>
      </Provider>
    );
  };

  return { store, ...render(ui, { wrapper: Wrapper, ...renderOptions }) };
};

// Mock WebSocket helper
export const createMockWebSocket = () => {
  const mockWs = {
    send: jest.fn(),
    close: jest.fn(),
    addEventListener: jest.fn(),
    removeEventListener: jest.fn(),
    readyState: WebSocket.OPEN,
    CONNECTING: WebSocket.CONNECTING,
    OPEN: WebSocket.OPEN,
    CLOSING: WebSocket.CLOSING,
    CLOSED: WebSocket.CLOSED,
  };

  return mockWs;
};

// Wait for async operations
export const waitForAsync = () => new Promise(resolve => setTimeout(resolve, 0));

// Mock fetch helper
export const mockFetch = (response: any, ok = true) => {
  global.fetch = jest.fn(() =>
    Promise.resolve({
      ok,
      json: () => Promise.resolve(response),
      text: () => Promise.resolve(JSON.stringify(response)),
      status: ok ? 200 : 500,
      statusText: ok ? 'OK' : 'Internal Server Error',
    } as Response)
  );
};

// Mock API response helper
export const createMockApiResponse = (data: any, ok = true) => ({
  ok,
  json: async () => data,
  status: ok ? 200 : 500,
  statusText: ok ? 'OK' : 'Error',
});

// Test data generators
export const generateMockRobotState = () => ({
  battery: 75,
  speed: 50,
  position: { x: 0, y: 0 },
  sensors: {
    ultrasonic: 50,
    lineTracking: { left: 0, center: 1, right: 0 },
  },
});

export const generateMockSystemMode = () => ({
  ok: true,
  mode: 0,
  mode_name: 'CAMERA_ONLY',
  description: 'Camera Only',
  manual_override: false,
  hybrid_mode: 'pi_only',
  orin_available: false,
  thermal_temp: 50,
  cpu_load: 30,
  throttling: false,
});

export const generateMockLatencyMetrics = () => ({
  piOnly: {
    total_processing_ms: 2.5,
    encode_duration_ms: 1.2,
    capture_to_encode_ms: 1.3,
  },
  hybrid: {
    round_trip_ms: { avg: 45.2, min: 40.0, max: 50.0, count: 100 },
    inference_ms: { avg: 12.3, min: 10.0, max: 15.0, count: 100 },
  },
});

// Re-export everything from testing-library
export * from '@testing-library/react';
export { default as userEvent } from '@testing-library/user-event';

