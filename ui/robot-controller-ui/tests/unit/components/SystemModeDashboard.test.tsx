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
    const mockModes = {
      0: { mode: 0, name: 'Camera Only', description: 'Camera Only', available: true },
      1: { mode: 1, name: 'Motion Detection', description: 'Motion Detection', available: true },
      2: { mode: 2, name: 'Tracking', description: 'Tracking', available: true },
      3: { mode: 3, name: 'Face Detection', description: 'Face Detection', available: true },
      4: { mode: 4, name: 'ArUco Detection', description: 'ArUco Detection', available: true },
      5: { mode: 5, name: 'Recording Only', description: 'Recording Only', available: true },
      6: { mode: 6, name: 'Orin-Enhanced', description: 'Orin-Enhanced', available: true },
      7: { mode: 7, name: 'Orin Navigation', description: 'Orin Navigation', available: true },
    };
    
    mockFetch({ ok: true, modes: mockModes, current_mode: 0 });
    mockFetch({ ok: true, ...generateMockSystemMode() });

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText('Camera Only')).toBeInTheDocument();
      expect(screen.getByText('Motion Detection')).toBeInTheDocument();
      expect(screen.getByText('Tracking')).toBeInTheDocument();
      expect(screen.getByText('Face Detection')).toBeInTheDocument();
      expect(screen.getByText('ArUco Detection')).toBeInTheDocument();
      expect(screen.getByText('Recording Only')).toBeInTheDocument();
      expect(screen.getByText('Orin-Enhanced')).toBeInTheDocument();
      expect(screen.getByText('Orin Navigation')).toBeInTheDocument();
    });
  });

  it('displays current mode status', async () => {
    const mockStatus = {
      ok: true,
      ...generateMockSystemMode(),
      mode: 3,
      mode_name: 'FACE_DETECTION',
      description: 'Face Detection mode',
    };
    const mockModes = {
      3: { mode: 3, name: 'Face Detection', description: 'Face Detection', available: true },
    };
    mockFetch({ ok: true, modes: mockModes, current_mode: 3 });
    mockFetch(mockStatus);

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/Mode 3: Face Detection/i)).toBeInTheDocument();
    });
  });

  it('handles mode switching', async () => {
    const user = require('@testing-library/user-event').default;
    const mockModes = {
      0: { mode: 0, name: 'Camera Only', description: 'Camera Only', available: true },
      1: { mode: 1, name: 'Motion Detection', description: 'Motion Detection', available: true },
    };
    mockFetch({ ok: true, modes: mockModes, current_mode: 0 });
    mockFetch({ ok: true, ...generateMockSystemMode() });

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText('Motion Detection')).toBeInTheDocument();
    });

    const button = screen.getByText('Motion Detection').closest('button');
    expect(button).toBeInTheDocument();
    
    if (button) {
      mockFetch({ ok: true, message: 'System mode set to 1', mode: 1, ...generateMockSystemMode() });
      await user.click(button);

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledWith(
          expect.stringContaining('/api/system/mode/set'),
          expect.objectContaining({
            method: 'POST',
          })
        );
      });
    }
  });

  it('displays thermal warnings when throttling', async () => {
    const mockStatus = {
      ok: true,
      ...generateMockSystemMode(),
      throttling: true,
      thermal_temp: 75,
      cpu_load: 80,
    };
    const mockModes = {
      0: { mode: 0, name: 'Camera Only', description: 'Camera Only', available: true },
      1: { mode: 1, name: 'Motion Detection', description: 'Motion Detection', available: true },
      2: { mode: 2, name: 'Tracking', description: 'Tracking', available: true },
      3: { mode: 3, name: 'Face Detection', description: 'Face Detection', available: true },
      4: { mode: 4, name: 'ArUco Detection', description: 'ArUco Detection', available: true },
      5: { mode: 5, name: 'Recording Only', description: 'Recording Only', available: true },
      6: { mode: 6, name: 'Orin-Enhanced', description: 'Orin-Enhanced', available: true },
      7: { mode: 7, name: 'Orin Navigation', description: 'Orin Navigation', available: true },
    };
    
    // Mock both API calls - list first, then status
    let callCount = 0;
    global.fetch = jest.fn((url: string) => {
      callCount++;
      if (url.includes('/list')) {
        return Promise.resolve({
          ok: true,
          json: () => Promise.resolve({ ok: true, modes: mockModes, current_mode: 0 }),
        } as Response);
      } else {
        return Promise.resolve({
          ok: true,
          json: () => Promise.resolve(mockStatus),
        } as Response);
      }
    });

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      expect(screen.getByText(/Throttling Active/i)).toBeInTheDocument();
    }, { timeout: 3000 });
  });

  it('handles API errors gracefully', async () => {
    // Mock fetch to return errors for both endpoints
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: false,
        json: () => Promise.resolve({ ok: false, error: 'API unavailable' }),
        status: 500,
        statusText: 'Internal Server Error',
      } as Response)
    );

    renderWithProviders(<SystemModeDashboard />);

    await waitFor(() => {
      // Component should still render even if API fails
      expect(screen.getByText(/System Mode Control/i)).toBeInTheDocument();
    }, { timeout: 3000 });
  });
});

