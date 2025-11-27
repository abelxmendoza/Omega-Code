/**
 * SystemModeDashboard Component Tests
 */

import React from 'react';
import { screen, waitFor } from '@testing-library/react';
import { renderWithProviders, mockFetch, generateMockSystemMode } from '../../utils/test-helpers';
import SystemModeDashboard from '@/components/SystemModeDashboard';

// Mock the API endpoints
jest.mock('next/router', () => ({
  useRouter: () => ({
    push: jest.fn(),
    pathname: '/',
    query: {},
  }),
}));

describe('SystemModeDashboard', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders all mode buttons', async () => {
    mockFetch({ ok: true, modes: {}, current_mode: 0 });
    mockFetch(generateMockSystemMode());

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText('0 Camera Only')).toBeInTheDocument();
      expect(screen.getByText('1 Motion Detection')).toBeInTheDocument();
      expect(screen.getByText('2 Tracking')).toBeInTheDocument();
      expect(screen.getByText('3 Face Detection')).toBeInTheDocument();
      expect(screen.getByText('4 ArUco Detection')).toBeInTheDocument();
      expect(screen.getByText('5 Recording Only')).toBeInTheDocument();
      expect(screen.getByText('6 Orin-Enhanced')).toBeInTheDocument();
      expect(screen.getByText('7 Orin Navigation')).toBeInTheDocument();
    });
  });

  it('displays current mode status', async () => {
    const mockStatus = {
      ...generateMockSystemMode(),
      mode: 3,
      mode_name: 'FACE_DETECTION',
    };
    mockFetch({ ok: true, modes: {}, current_mode: 3 });
    mockFetch(mockStatus);

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/Mode 3: Face Detection/i)).toBeInTheDocument();
    });
  });

  it('handles mode switching', async () => {
    const user = require('@testing-library/user-event').default;
    mockFetch({ ok: true, modes: {}, current_mode: 0 });
    mockFetch(generateMockSystemMode());

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      const button = screen.getByText('1 Motion Detection');
      expect(button).toBeInTheDocument();
    });

    const button = screen.getByText('1 Motion Detection');
    mockFetch({ ok: true, message: 'System mode set to 1', mode: 1 });
    
    await user.click(button);

    await waitFor(() => {
      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('/api/system/mode/set'),
        expect.objectContaining({
          method: 'POST',
          body: JSON.stringify({ mode: 1 }),
        })
      );
    });
  });

  it('displays thermal warnings when throttling', async () => {
    const mockStatus = {
      ...generateMockSystemMode(),
      throttling: true,
      thermal_temp: 75,
      cpu_load: 80,
    };
    mockFetch({ ok: true, modes: {}, current_mode: 0 });
    mockFetch(mockStatus);

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/throttling/i)).toBeInTheDocument();
    });
  });

  it('handles API errors gracefully', async () => {
    mockFetch({ ok: false, error: 'API unavailable' }, false);
    mockFetch({ ok: false, error: 'API unavailable' }, false);

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      // Component should still render even if API fails
      expect(screen.getByText(/System Mode Control/i)).toBeInTheDocument();
    });
  });
});

