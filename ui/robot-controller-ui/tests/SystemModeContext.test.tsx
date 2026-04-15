/**
 * Tests for SystemModeContext (provider + consumer hook).
 *
 * Verifies:
 *   - Provider supplies initial state to consumers
 *   - State updates on successful fetch
 *   - Offline flag set after FAIL_THRESHOLD consecutive failures
 *   - refresh() triggers an immediate refetch
 *   - useSystemModeContext() throws outside the provider
 */

import React from 'react';
import { render, screen, act, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import userEvent from '@testing-library/user-event';
import {
  SystemModeProvider,
  useSystemModeContext,
} from '../src/context/SystemModeContext';

// ── Mock data ──────────────────────────────────────────────────────────────────

const MOCK_STATUS = {
  mode:           2,
  mode_name:      'AUTONOMOUS',
  description:    'Autonomous mode',
  manual_override: false,
  orin_available:  true,
  hybrid_mode:    'orin_led',
  thermal_temp:   55,
  cpu_load:       40,
  throttling:     false,
};

// ── Consumer helper ────────────────────────────────────────────────────────────

/** Renders a tiny consumer component so we can inspect context values. */
function Consumer() {
  const ctx = useSystemModeContext();
  return (
    <div>
      <span data-testid="mode">{String(ctx.mode)}</span>
      <span data-testid="modeName">{ctx.modeName}</span>
      <span data-testid="offline">{String(ctx.offline)}</span>
      <span data-testid="orin">{String(ctx.orinAvailable)}</span>
      <span data-testid="thermal">{String(ctx.thermalTemp)}</span>
      <button onClick={ctx.refresh}>Refresh</button>
    </div>
  );
}

// ── Helpers ────────────────────────────────────────────────────────────────────

function mockFetchOk(data: object) {
  global.fetch = jest.fn().mockResolvedValue({
    ok: true,
    status: 200,
    json: () => Promise.resolve(data),
  } as Response);
}

function mockFetchError() {
  global.fetch = jest.fn().mockResolvedValue({
    ok: false,
    status: 503,
    json: () => Promise.resolve({}),
  } as Response);
}

// ── Tests ──────────────────────────────────────────────────────────────────────

describe('SystemModeContext', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  afterEach(() => {
    jest.clearAllTimers();
  });

  describe('initial state', () => {
    it('provides null mode and offline=false before first fetch', async () => {
      // Block the fetch so it never resolves during this test
      global.fetch = jest.fn(() => new Promise(() => {}));

      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      expect(screen.getByTestId('mode').textContent).toBe('null');
      expect(screen.getByTestId('offline').textContent).toBe('false');
    });
  });

  describe('successful fetch', () => {
    it('updates mode and modeName from API response', async () => {
      mockFetchOk(MOCK_STATUS);

      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      await waitFor(() => {
        expect(screen.getByTestId('mode').textContent).toBe('2');
        expect(screen.getByTestId('modeName').textContent).toBe('AUTONOMOUS');
      });
    });

    it('sets orinAvailable from response', async () => {
      mockFetchOk(MOCK_STATUS);

      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      await waitFor(() => {
        expect(screen.getByTestId('orin').textContent).toBe('true');
      });
    });

    it('sets thermalTemp from response', async () => {
      mockFetchOk(MOCK_STATUS);

      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      await waitFor(() => {
        expect(screen.getByTestId('thermal').textContent).toBe('55');
      });
    });

    it('keeps offline=false after a successful fetch', async () => {
      mockFetchOk(MOCK_STATUS);

      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      await waitFor(() => {
        expect(screen.getByTestId('offline').textContent).toBe('false');
      });
    });
  });

  describe('offline / failure state', () => {
    it('sets offline=true after 3 consecutive failures', async () => {
      mockFetchError();

      jest.useFakeTimers();
      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      // First fetch (on mount) — fails → failCount = 1
      await act(async () => { await Promise.resolve(); });

      // Advance past two more 15 s poll intervals → failCount reaches 3
      await act(async () => { jest.advanceTimersByTime(15_000); });
      await act(async () => { await Promise.resolve(); });
      await act(async () => { jest.advanceTimersByTime(15_000); });
      await act(async () => { await Promise.resolve(); });

      expect(screen.getByTestId('offline').textContent).toBe('true');
      jest.useRealTimers();
    });
  });

  describe('refresh()', () => {
    it('calls fetch again when refresh is triggered', async () => {
      mockFetchOk(MOCK_STATUS);
      const user = userEvent.setup();

      await act(async () => {
        render(
          <SystemModeProvider>
            <Consumer />
          </SystemModeProvider>,
        );
      });

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledTimes(1);
      });

      await user.click(screen.getByRole('button', { name: 'Refresh' }));

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledTimes(2);
      });
    });
  });

  describe('hook guard', () => {
    it('throws when used outside <SystemModeProvider>', () => {
      // Suppress the React error boundary console noise
      const spy = jest.spyOn(console, 'error').mockImplementation(() => {});

      function BadConsumer() {
        useSystemModeContext(); // should throw
        return <div />;
      }

      expect(() => render(<BadConsumer />)).toThrow(
        'useSystemModeContext must be used inside <SystemModeProvider>',
      );

      spy.mockRestore();
    });
  });

  describe('multiple consumers', () => {
    it('all consumers share the same fetch result (single request)', async () => {
      mockFetchOk(MOCK_STATUS);

      function ConsumerA() {
        const { mode } = useSystemModeContext();
        return <span data-testid="a">{String(mode)}</span>;
      }
      function ConsumerB() {
        const { modeName } = useSystemModeContext();
        return <span data-testid="b">{modeName}</span>;
      }

      await act(async () => {
        render(
          <SystemModeProvider>
            <ConsumerA />
            <ConsumerB />
          </SystemModeProvider>,
        );
      });

      await waitFor(() => {
        expect(screen.getByTestId('a').textContent).toBe('2');
        expect(screen.getByTestId('b').textContent).toBe('AUTONOMOUS');
      });

      // Both consumers share the same provider → only one fetch needed
      expect(global.fetch).toHaveBeenCalledTimes(1);
    });
  });
});
