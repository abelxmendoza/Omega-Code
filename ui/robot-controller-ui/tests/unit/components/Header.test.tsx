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
    const mockLatency = {
      ok: true,
      type: 'pi_only',
      latencies_ms: generateMockLatencyMetrics().piOnly,
    };
    mockFetch(mockLatency);

    renderWithProviders(<Header batteryLevel={75} />);

    await waitFor(() => {
      expect(screen.getByText(/Pi:/i)).toBeInTheDocument();
      expect(screen.getByText(/2.5ms/i)).toBeInTheDocument();
    }, { timeout: 2000 });
  });

  it('displays service status indicators', () => {
    renderWithProviders(<Header batteryLevel={75} />);
    expect(screen.getByText(/Pi:/i)).toBeInTheDocument();
    expect(screen.getByText(/Status:/i)).toBeInTheDocument();
  });

  it('shows network profile badge', () => {
    renderWithProviders(<Header batteryLevel={75} />);
    // Profile badge should be visible
    expect(screen.getByText(/TAILSCALE|LAN|LOCAL/i)).toBeInTheDocument();
  });
});

