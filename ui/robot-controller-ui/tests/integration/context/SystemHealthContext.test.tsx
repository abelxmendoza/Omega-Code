/*
# File: tests/integration/context/SystemHealthContext.test.tsx
# Tests: SystemHealthProvider + useSystemHealth()
#
# Covers:
#   1. Provider renders children without throwing
#   2. useSystemHealth() returns initial state (backend=down, connected=false)
#   3. State updates when the service notifies subscribers
#   4. useSystemHealth() throws when called outside a provider
#   5. Provider starts and stops the health service on mount/unmount
*/

import React from 'react';
import { render, act, screen } from '@testing-library/react';
import { SystemHealthProvider, useSystemHealth } from '@/context/SystemHealthContext';
import { systemHealth } from '@/services/systemHealth';
import type { SystemHealthState } from '@/services/systemHealth';

// ─── Test component that reads from context ──────────────────────────────────

function HealthReader() {
  const health = useSystemHealth();
  return (
    <div>
      <span data-testid="connected">{String(health.connected)}</span>
      <span data-testid="degraded">{String(health.degraded)}</span>
      <span data-testid="backend">{health.components.backend}</span>
      <span data-testid="updated">{String(health.lastUpdated)}</span>
    </div>
  );
}

// ─── Setup ──────────────────────────────────────────────────────────────────

let startSpy: jest.SpyInstance;
let stopSpy: jest.SpyInstance;
let subscribeSpy: jest.SpyInstance;
let capturedSubscriber: ((s: SystemHealthState) => void) | null = null;

beforeEach(() => {
  capturedSubscriber = null;

  // Prevent the real singleton from firing HTTP requests in tests
  startSpy     = jest.spyOn(systemHealth, 'start').mockImplementation(() => {});
  stopSpy      = jest.spyOn(systemHealth, 'stop').mockImplementation(() => {});
  subscribeSpy = jest.spyOn(systemHealth, 'subscribe').mockImplementation((cb) => {
    capturedSubscriber = cb;
    cb(systemHealth.getState()); // mirror real behaviour: immediate emit on subscribe
    return () => { capturedSubscriber = null; };
  });
});

afterEach(() => {
  jest.restoreAllMocks();
});

// ─────────────────────────────────────────────────────────────────────────────
// Render
// ─────────────────────────────────────────────────────────────────────────────

describe('SystemHealthProvider renders', () => {
  it('renders children without throwing', () => {
    render(
      <SystemHealthProvider>
        <div data-testid="child">hello</div>
      </SystemHealthProvider>
    );
    expect(screen.getByTestId('child')).toBeInTheDocument();
  });

  it('provides initial disconnected state to consumers', () => {
    render(
      <SystemHealthProvider>
        <HealthReader />
      </SystemHealthProvider>
    );
    expect(screen.getByTestId('connected').textContent).toBe('false');
    expect(screen.getByTestId('backend').textContent).toBe('down');
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle — start / stop
// ─────────────────────────────────────────────────────────────────────────────

describe('SystemHealthProvider lifecycle', () => {
  it('starts the health service on mount', () => {
    render(
      <SystemHealthProvider>
        <div />
      </SystemHealthProvider>
    );
    expect(startSpy).toHaveBeenCalledTimes(1);
  });

  it('stops the health service on unmount', () => {
    const { unmount } = render(
      <SystemHealthProvider>
        <div />
      </SystemHealthProvider>
    );
    unmount();
    expect(stopSpy).toHaveBeenCalledTimes(1);
  });

  it('subscribes to the health service on mount', () => {
    render(
      <SystemHealthProvider>
        <div />
      </SystemHealthProvider>
    );
    expect(subscribeSpy).toHaveBeenCalledTimes(1);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// State updates
// ─────────────────────────────────────────────────────────────────────────────

describe('state propagation', () => {
  it('re-renders consumers when the service emits a connected state', () => {
    render(
      <SystemHealthProvider>
        <HealthReader />
      </SystemHealthProvider>
    );
    // Simulate backend coming online
    act(() => {
      capturedSubscriber!({
        connected: true,
        degraded: false,
        lastUpdated: 1234567890,
        components: { backend: 'ok', camera: 'ok', sensors: 'ok', performance: 'ok' },
      });
    });
    expect(screen.getByTestId('connected').textContent).toBe('true');
    expect(screen.getByTestId('backend').textContent).toBe('ok');
  });

  it('re-renders consumers when the service emits a degraded state', () => {
    render(
      <SystemHealthProvider>
        <HealthReader />
      </SystemHealthProvider>
    );
    act(() => {
      capturedSubscriber!({
        connected: false,
        degraded: true,
        lastUpdated: Date.now(),
        components: { backend: 'down', camera: 'down', sensors: 'down', performance: 'down' },
      });
    });
    expect(screen.getByTestId('connected').textContent).toBe('false');
    expect(screen.getByTestId('degraded').textContent).toBe('true');
    expect(screen.getByTestId('backend').textContent).toBe('down');
  });

  it('updates back to connected after recovery from degraded', () => {
    render(
      <SystemHealthProvider>
        <HealthReader />
      </SystemHealthProvider>
    );
    // First: degraded
    act(() => {
      capturedSubscriber!({
        connected: false,
        degraded: true,
        lastUpdated: Date.now(),
        components: { backend: 'down', camera: 'down', sensors: 'down', performance: 'down' },
      });
    });
    expect(screen.getByTestId('degraded').textContent).toBe('true');

    // Then: recovered
    act(() => {
      capturedSubscriber!({
        connected: true,
        degraded: false,
        lastUpdated: Date.now(),
        components: { backend: 'ok', camera: 'ok', sensors: 'ok', performance: 'ok' },
      });
    });
    expect(screen.getByTestId('connected').textContent).toBe('true');
    expect(screen.getByTestId('degraded').textContent).toBe('false');
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Error boundary — missing provider
// ─────────────────────────────────────────────────────────────────────────────

describe('useSystemHealth outside provider', () => {
  it('throws a descriptive error when called outside SystemHealthProvider', () => {
    // Suppress the React error boundary console.error noise in the test output
    const consoleSpy = jest.spyOn(console, 'error').mockImplementation(() => {});
    expect(() => render(<HealthReader />)).toThrow(
      'useSystemHealth must be used within a <SystemHealthProvider>'
    );
    consoleSpy.mockRestore();
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Single polling loop guarantee
// ─────────────────────────────────────────────────────────────────────────────

describe('single polling loop guarantee', () => {
  it('only calls systemHealth.start() once even with multiple consumers', () => {
    render(
      <SystemHealthProvider>
        <HealthReader />
        <HealthReader />
        <HealthReader />
      </SystemHealthProvider>
    );
    // The service is a singleton — start() is called once at the provider level,
    // not once per consumer component.
    expect(startSpy).toHaveBeenCalledTimes(1);
  });
});
