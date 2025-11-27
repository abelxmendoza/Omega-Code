/**
 * System Mode Component Integration Tests
 * Tests component integration with API
 */

import React from 'react';
import { screen, waitFor } from '@testing-library/react';
import { renderWithProviders, mockFetch } from '../../utils/test-helpers';
import SystemModeDashboard from '@/components/SystemModeDashboard';

describe('SystemModeDashboard Integration', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('integrates with API endpoints', async () => {
    // Mock API responses
    mockFetch({ ok: true, modes: {}, current_mode: 0 });
    mockFetch({
      ok: true,
      mode: 0,
      description: 'Camera Only',
      manual_override: false,
      hybrid_mode: 'pi_only',
    });

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/System Mode Control/i)).toBeInTheDocument();
    });

    // Verify API was called
    expect(global.fetch).toHaveBeenCalledWith(
      expect.stringContaining('/api/system/mode/list'),
      expect.any(Object)
    );
  });

  it('handles API errors gracefully', async () => {
    mockFetch({ ok: false, error: 'API unavailable' }, false);

    renderWithProviders(<SystemModeDashboard />);

    // Component should still render
    await waitFor(() => {
      expect(screen.getByText(/System Mode Control/i)).toBeInTheDocument();
    });
  });

  it('updates status on mode change', async () => {
    const user = require('@testing-library/user-event').default;
    
    mockFetch({ ok: true, modes: {}, current_mode: 0 });
    mockFetch({
      ok: true,
      mode: 0,
      description: 'Camera Only',
    });

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText('1 Motion Detection')).toBeInTheDocument();
    });

    // Mock successful mode change
    mockFetch({ ok: true, message: 'Mode set', mode: 1 });
    mockFetch({
      ok: true,
      mode: 1,
      description: 'Motion Detection',
    });

    const button = screen.getByText('1 Motion Detection');
    await user.click(button);

    await waitFor(() => {
      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('/api/system/mode/set'),
        expect.objectContaining({
          method: 'POST',
        })
      );
    });
  });
});

