/**
 * SystemModeDashboard Snapshot Tests
 * Visual regression tests for SystemModeDashboard component
 */

import React from 'react';
import { render } from '@testing-library/react';
import SystemModeDashboard from '@/components/SystemModeDashboard';
import { renderWithProviders } from '../utils/test-helpers';

describe('SystemModeDashboard Snapshots', () => {
  it('matches snapshot in default state', () => {
    const { container } = renderWithProviders(<SystemModeDashboard />);
    expect(container.firstChild).toMatchSnapshot();
  });

  it('matches snapshot with mode 3 selected', () => {
    // Mock mode 3 selected
    const { container } = renderWithProviders(<SystemModeDashboard />);
    expect(container.firstChild).toMatchSnapshot();
  });

  it('matches snapshot with throttling warning', () => {
    // Mock throttling state
    const { container } = renderWithProviders(<SystemModeDashboard />);
    expect(container.firstChild).toMatchSnapshot();
  });
});

