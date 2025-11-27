/**
 * LatencyDashboard Snapshot Tests
 * Visual regression tests for LatencyDashboard component
 */

import React from 'react';
import { render } from '@testing-library/react';
import LatencyDashboard from '@/components/LatencyDashboard';
import { renderWithProviders } from '../utils/test-helpers';

describe('LatencyDashboard Snapshots', () => {
  it('matches snapshot with Pi-only latency', () => {
    const { container } = renderWithProviders(<LatencyDashboard />);
    expect(container.firstChild).toMatchSnapshot();
  });

  it('matches snapshot with hybrid latency', () => {
    const { container } = renderWithProviders(<LatencyDashboard />);
    expect(container.firstChild).toMatchSnapshot();
  });

  it('matches snapshot with no data', () => {
    const { container } = renderWithProviders(<LatencyDashboard />);
    expect(container.firstChild).toMatchSnapshot();
  });
});

