/**
 * Tests for LocalizationPanel component.
 *
 * The panel polls GET /localization/pose every 2 s and renders:
 *   - Offline state when backend unreachable
 *   - Rate-limited state on HTTP 429
 *   - Pose data (x, y, θ, quality, marker) on success
 */

import React from 'react';
import { screen, act, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import LocalizationPanel from '../src/components/LocalizationPanel';

// ── Mock data ──────────────────────────────────────────────────────────────────

const MOCK_POSE = {
  x:                1.234,
  y:               -0.567,
  theta_rad:        0.785,
  theta_deg:        45.0,
  covariance:       [[0.04, 0, 0], [0, 0.04, 0], [0, 0, 0.02]],
  quality:          0.82,
  correction_count: 5,
  last_marker_seen: 7,
  ts:               12345.0,
};

const MOCK_POSE_POOR = {
  ...MOCK_POSE,
  quality: 0.1,
  correction_count: 0,
  last_marker_seen: null,
};

const MOCK_POSE_FAIR = {
  ...MOCK_POSE,
  quality: 0.5,
};

// ── Helpers ────────────────────────────────────────────────────────────────────

function mockFetchOk(data: object) {
  global.fetch = jest.fn().mockResolvedValue({
    ok: true,
    status: 200,
    json: () => Promise.resolve(data),
  } as unknown as Response);
}

function mockFetchError(status = 500) {
  global.fetch = jest.fn().mockResolvedValue({
    ok: status < 400 || status >= 500 ? false : false,
    status,
    json: () => Promise.resolve({ error: 'failed' }),
  } as unknown as Response);
}

function mockFetchNetworkError() {
  global.fetch = jest.fn().mockRejectedValue(new Error('Network error'));
}

// ── Tests ──────────────────────────────────────────────────────────────────────

describe('LocalizationPanel', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // jsdom doesn't implement AbortSignal.timeout — stub it so the component
    // can construct the fetch signal without throwing before the mock is reached.
    if (!(AbortSignal as any).timeout) {
      (AbortSignal as any).timeout = (_ms: number) => new AbortController().signal;
    }
  });

  afterEach(() => {
    jest.clearAllTimers();
  });

  describe('offline state', () => {
    it('shows "offline" message when backend returns error', async () => {
      mockFetchError();
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText(/Localization offline/i)).toBeInTheDocument();
      });
    });

    it('shows "offline" message on network failure', async () => {
      mockFetchNetworkError();
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText(/Localization offline/i)).toBeInTheDocument();
      });
    });

    it('still shows stats panel and canvas when offline', async () => {
      mockFetchError();
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        // Offline banner present
        expect(screen.getByText(/Localization offline/i)).toBeInTheDocument();
        // Zeroed stats are always rendered (X and Y both show 0.000 m)
        expect(screen.getAllByText('0.000 m').length).toBeGreaterThanOrEqual(2);
        // Canvas always present
        expect(document.querySelector('canvas')).toBeInTheDocument();
      });
    });
  });

  describe('rate limited state', () => {
    it('shows rate-limited banner on HTTP 429', async () => {
      mockFetchError(429);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText(/Rate limited/i)).toBeInTheDocument();
      });
    });
  });

  describe('online — pose data rendered', () => {
    it('renders the panel heading', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('Localization')).toBeInTheDocument();
      });
    });

    it('renders x position in metres', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('1.234 m')).toBeInTheDocument();
      });
    });

    it('renders y position in metres', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('-0.567 m')).toBeInTheDocument();
      });
    });

    it('renders heading in degrees', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('45.0°')).toBeInTheDocument();
      });
    });

    it('renders correction count (ArUco fixes)', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('5')).toBeInTheDocument();
      });
    });

    it('renders last marker ID', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('#7')).toBeInTheDocument();
      });
    });

    it('shows "—" when no marker has been seen', async () => {
      mockFetchOk(MOCK_POSE_POOR);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        // Marker column shows "—" when last_marker_seen is null
        const dashes = screen.getAllByText('—');
        expect(dashes.length).toBeGreaterThan(0);
      });
    });
  });

  describe('quality badge', () => {
    it('shows "Good" label for quality >= 0.7', async () => {
      mockFetchOk(MOCK_POSE); // quality 0.82
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('Good')).toBeInTheDocument();
      });
    });

    it('shows "Fair" label for quality in [0.35, 0.7)', async () => {
      mockFetchOk(MOCK_POSE_FAIR); // quality 0.5
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('Fair')).toBeInTheDocument();
      });
    });

    it('shows "Poor" label for quality in (0, 0.35)', async () => {
      mockFetchOk(MOCK_POSE_POOR); // quality 0.1
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText('Poor')).toBeInTheDocument();
      });
    });

    it('shows quality percentage', async () => {
      mockFetchOk(MOCK_POSE); // quality 0.82 → 82%
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText(/82%/i)).toBeInTheDocument();
      });
    });
  });

  describe('dead-reckoning hint', () => {
    it('shows dead-reckoning hint when correction_count is 0 (live data)', async () => {
      mockFetchOk(MOCK_POSE_POOR); // correction_count 0
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText(/Dead-reckoning only/i)).toBeInTheDocument();
      });
    });

    it('shows dead-reckoning hint when offline (placeholder correction_count = 0)', async () => {
      mockFetchError();
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.getByText(/Dead-reckoning only/i)).toBeInTheDocument();
      });
    });

    it('does NOT show dead-reckoning hint when corrections > 0', async () => {
      mockFetchOk(MOCK_POSE); // correction_count 5
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        expect(screen.queryByText(/Dead-reckoning only/i)).not.toBeInTheDocument();
      });
    });
  });

  describe('canvas', () => {
    it('renders a canvas element immediately (even when offline)', async () => {
      mockFetchError();
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      // Canvas is always present — no need to wait for live data
      const canvas = document.querySelector('canvas');
      expect(canvas).toBeInTheDocument();
    });

    it('renders a canvas element when pose data is available', async () => {
      mockFetchOk(MOCK_POSE);
      await act(async () => {
        renderWithProviders(<LocalizationPanel />);
      });

      await waitFor(() => {
        const canvas = document.querySelector('canvas');
        expect(canvas).toBeInTheDocument();
      });
    });
  });
});
