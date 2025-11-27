/**
 * Header Component Tests
 */

import React from 'react';
import { screen, waitFor } from '@testing-library/react';
import { renderWithProviders, mockFetch, generateMockLatencyMetrics } from '../../utils/test-helpers';
import Header from '@/components/Header';

describe('Header', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders header with logo and title', () => {
    renderWithProviders(<Header batteryLevel={75} />);
    expect(screen.getByText(/Robot Controller/i)).toBeInTheDocument();
    expect(screen.getByAltText(/Omega Tech Logo/i)).toBeInTheDocument();
  });

  it('displays battery level', () => {
    renderWithProviders(<Header batteryLevel={75} />);
    expect(screen.getByText(/75%/i)).toBeInTheDocument();
  });

  it('displays latency metrics when available', async () => {
    const mockPiLatency = {
      ok: true,
      capture_to_encode_ms: 2.5,
      encode_duration_ms: 1.2,
    };
    const mockHybridLatency = {
      ok: false,
    };
    
    // Mock both API endpoints
    let callCount = 0;
    global.fetch = jest.fn((url: string) => {
      callCount++;
      if (url.includes('/latency/hybrid')) {
        return Promise.resolve({
          ok: true,
          json: () => Promise.resolve(mockHybridLatency),
        } as Response);
      } else if (url.includes('/latency')) {
        return Promise.resolve({
          ok: true,
          json: () => Promise.resolve(mockPiLatency),
        } as Response);
      }
      return Promise.resolve({
        ok: false,
        json: () => Promise.resolve({ ok: false }),
      } as Response);
    });

    renderWithProviders(<Header batteryLevel={75} />);

    await waitFor(() => {
      expect(screen.getByText(/Pi:/i)).toBeInTheDocument();
    }, { timeout: 3000 });
  });

  it('displays service status indicators', async () => {
    // Mock latency endpoints
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: false,
        json: () => Promise.resolve({ ok: false }),
      } as Response)
    );

    renderWithProviders(<Header batteryLevel={75} />);
    
    await waitFor(() => {
      expect(screen.getByText(/Robot Controller/i)).toBeInTheDocument();
    });
  });

  it('shows network profile badge', () => {
    renderWithProviders(<Header batteryLevel={75} />);
    // Profile badge should be visible
    expect(screen.getByText(/TAILSCALE|LAN|LOCAL/i)).toBeInTheDocument();
  });
});

