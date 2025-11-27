/**
 * LatencyDashboard Component Tests
 */

import React from 'react';
import { screen, waitFor } from '@testing-library/react';
import { renderWithProviders, mockFetch, generateMockLatencyMetrics } from '../../utils/test-helpers';
import LatencyDashboard from '@/components/LatencyDashboard';

describe('LatencyDashboard', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders latency dashboard', () => {
    renderWithProviders(<LatencyDashboard />);
    expect(screen.getByText(/Latency Metrics/i)).toBeInTheDocument();
  });

  it('displays Pi-only latency metrics', async () => {
    const mockLatency = {
      ok: true,
      type: 'pi_only',
      latencies_ms: generateMockLatencyMetrics().piOnly,
    };
    mockFetch(mockLatency);

    renderWithProviders(<LatencyDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/Pi-Only Latency/i)).toBeInTheDocument();
      expect(screen.getByText(/2.5.*ms/i)).toBeInTheDocument(); // Total processing
    });
  });

  it('displays hybrid latency metrics when available', async () => {
    const mockPiLatency = {
      ok: true,
      type: 'pi_only',
      latencies_ms: generateMockLatencyMetrics().piOnly,
    };
    const mockHybridLatency = {
      ok: true,
      type: 'hybrid',
      round_trip_ms: generateMockLatencyMetrics().hybrid.round_trip_ms,
      inference_ms: generateMockLatencyMetrics().hybrid.inference_ms,
    };

    // Mock sequential calls
    let callCount = 0;
    global.fetch = jest.fn(() => {
      callCount++;
      if (callCount === 1) {
        return Promise.resolve({
          ok: true,
          json: () => Promise.resolve(mockPiLatency),
        } as Response);
      } else {
        return Promise.resolve({
          ok: true,
          json: () => Promise.resolve(mockHybridLatency),
        } as Response);
      }
    });

    renderWithProviders(<LatencyDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/Pi ↔ Orin Round-Trip/i)).toBeInTheDocument();
      expect(screen.getByText(/45.2.*ms/i)).toBeInTheDocument(); // Round-trip avg
    }, { timeout: 3000 });
  });

  it('handles missing latency data gracefully', async () => {
    mockFetch({ ok: false, error: 'No latency data available' }, false);

    renderWithProviders(<LatencyDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/No latency data available/i)).toBeInTheDocument();
    });
  });

  it('updates latency metrics in real-time', async () => {
    const mockLatency = {
      ok: true,
      type: 'pi_only',
      latencies_ms: { total_processing_ms: 2.5 },
    };
    mockFetch(mockLatency);

    renderWithProviders(<LatencyDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/2.5.*ms/i)).toBeInTheDocument();
    });

    // Simulate update with new value
    const updatedLatency = {
      ok: true,
      type: 'pi_only',
      latencies_ms: { total_processing_ms: 3.0 },
    };
    mockFetch(updatedLatency);

    // Wait for next poll (500ms interval)
    await waitFor(() => {
      expect(screen.getByText(/3.0.*ms/i)).toBeInTheDocument();
    }, { timeout: 1000 });
  });
});

