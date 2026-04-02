/*
# File: tests/unit/services/systemHealth.test.ts
# Tests: systemHealth singleton — circuit breaker, state transitions, subscriber pattern
#
# Strategy: bypass the timer scheduling and call probe() directly.
# This avoids fake-timer complexity while still testing all state logic.
#
# Circuit breaker invariants:
#   - 3 consecutive failures → degraded=true, backend='down'
#   - Any success resets failureCount + restores connected=true
#   - Subscriber is notified on every probe
#   - subscribe() immediately emits current state
*/

import {
  systemHealth,
  FAILURE_THRESHOLD,
  POLL_INTERVAL_MS,
  BACKOFF_INTERVAL_MS,
} from '@/services/systemHealth';

// ─── Helpers ────────────────────────────────────────────────────────────────

/** Reset all private fields of the singleton to a clean slate. */
function resetService() {
  systemHealth.stop();
  const s = systemHealth as any;
  s.failureCount     = 0;
  s.activeInterval   = POLL_INTERVAL_MS;
  s.running          = false;
  s.timerId          = null;
  s.state = {
    connected: false,
    degraded: false,
    lastUpdated: 0,
    components: { backend: 'down', camera: 'down', sensors: 'down', performance: 'down' },
  };
  s.subscribers      = new Set();
}

/** Call the private probe() method directly (no timer needed). */
async function triggerProbe(): Promise<void> {
  const s = systemHealth as any;
  s.running = true; // probe guards on this flag
  await (s.probe as () => Promise<void>).call(s);
}

// ─── Setup ──────────────────────────────────────────────────────────────────

beforeEach(() => {
  resetService();
  jest.useFakeTimers();
});

afterEach(() => {
  systemHealth.stop();
  jest.useRealTimers();
  jest.restoreAllMocks();
});

// ─────────────────────────────────────────────────────────────────────────────
// Success path
// ─────────────────────────────────────────────────────────────────────────────

describe('successful health probe', () => {
  it('sets connected=true and backend=ok', async () => {
    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    await triggerProbe();
    const state = systemHealth.getState();
    expect(state.connected).toBe(true);
    expect(state.degraded).toBe(false);
    expect(state.components.backend).toBe('ok');
  });

  it('updates lastUpdated timestamp', async () => {
    const before = Date.now();
    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    await triggerProbe();
    expect(systemHealth.getState().lastUpdated).toBeGreaterThanOrEqual(before);
  });

  it('resets failureCount to 0', async () => {
    (systemHealth as any).failureCount = 2;
    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    await triggerProbe();
    expect((systemHealth as any).failureCount).toBe(0);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Failure path
// ─────────────────────────────────────────────────────────────────────────────

describe('failed health probe', () => {
  it('sets connected=false on network error', async () => {
    jest.spyOn(global, 'fetch').mockRejectedValue(new Error('Network error'));
    await triggerProbe();
    expect(systemHealth.getState().connected).toBe(false);
  });

  it('increments failureCount on each failure', async () => {
    jest.spyOn(global, 'fetch').mockRejectedValue(new Error('fail'));
    await triggerProbe();
    expect((systemHealth as any).failureCount).toBe(1);
    await triggerProbe();
    expect((systemHealth as any).failureCount).toBe(2);
  });

  it('sets backend=degraded before threshold is reached', async () => {
    jest.spyOn(global, 'fetch').mockRejectedValue(new Error('fail'));
    await triggerProbe(); // failureCount = 1 (below threshold)
    expect(systemHealth.getState().components.backend).toBe('degraded');
    expect(systemHealth.getState().degraded).toBe(false);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Circuit breaker — threshold
// ─────────────────────────────────────────────────────────────────────────────

describe('circuit breaker threshold', () => {
  it(`marks degraded=true after ${FAILURE_THRESHOLD} consecutive failures`, async () => {
    jest.spyOn(global, 'fetch').mockRejectedValue(new Error('fail'));
    for (let i = 0; i < FAILURE_THRESHOLD; i++) {
      await triggerProbe();
    }
    expect(systemHealth.getState().degraded).toBe(true);
    expect(systemHealth.getState().components.backend).toBe('down');
  });

  it('escalates activeInterval to BACKOFF_INTERVAL_MS at threshold', async () => {
    // Pre-set failureCount to just below threshold so one more failure trips it
    (systemHealth as any).failureCount = FAILURE_THRESHOLD - 1;
    jest.spyOn(global, 'fetch').mockRejectedValue(new Error('fail'));
    await triggerProbe();
    expect((systemHealth as any).activeInterval).toBe(BACKOFF_INTERVAL_MS);
  });

  it('does NOT escalate before threshold is reached', async () => {
    jest.spyOn(global, 'fetch').mockRejectedValue(new Error('fail'));
    // One failure below threshold
    await triggerProbe();
    expect((systemHealth as any).activeInterval).toBe(POLL_INTERVAL_MS);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Circuit breaker — recovery
// ─────────────────────────────────────────────────────────────────────────────

describe('circuit breaker recovery', () => {
  it('resets degraded state on successful probe after failures', async () => {
    // Simulate backed-off state
    (systemHealth as any).failureCount = FAILURE_THRESHOLD;
    (systemHealth as any).activeInterval = BACKOFF_INTERVAL_MS;
    (systemHealth as any).state.degraded = true;

    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    await triggerProbe();

    expect(systemHealth.getState().connected).toBe(true);
    expect(systemHealth.getState().degraded).toBe(false);
  });

  it('restores activeInterval to POLL_INTERVAL_MS after recovery', async () => {
    (systemHealth as any).failureCount = FAILURE_THRESHOLD;
    (systemHealth as any).activeInterval = BACKOFF_INTERVAL_MS;

    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    await triggerProbe();

    expect((systemHealth as any).activeInterval).toBe(POLL_INTERVAL_MS);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Subscriber pattern
// ─────────────────────────────────────────────────────────────────────────────

describe('subscribe / unsubscribe', () => {
  it('immediately emits current state on subscribe', () => {
    const cb = jest.fn();
    systemHealth.subscribe(cb);
    expect(cb).toHaveBeenCalledTimes(1);
    expect(cb).toHaveBeenCalledWith(systemHealth.getState());
  });

  it('notifies subscriber after each probe', async () => {
    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    const cb = jest.fn();
    systemHealth.subscribe(cb);
    cb.mockClear(); // ignore the immediate emit
    await triggerProbe();
    expect(cb).toHaveBeenCalledTimes(1);
  });

  it('does not call callback after unsubscribe', async () => {
    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    const cb = jest.fn();
    const unsub = systemHealth.subscribe(cb);
    cb.mockClear();
    unsub();
    await triggerProbe();
    expect(cb).not.toHaveBeenCalled();
  });

  it('passes a state snapshot (not the live object)', () => {
    const cb = jest.fn();
    systemHealth.subscribe(cb);
    const received = cb.mock.calls[0][0];
    // Mutate the live state — snapshot should be independent
    (systemHealth as any).state.connected = !received.connected;
    expect(received.connected).not.toBe(systemHealth.getState().connected);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// start / stop lifecycle
// ─────────────────────────────────────────────────────────────────────────────

describe('start / stop', () => {
  it('does not start twice if already running', () => {
    jest.spyOn(global, 'fetch').mockResolvedValue({ ok: true } as Response);
    systemHealth.start();
    const firstTimerId = (systemHealth as any).timerId;
    systemHealth.start(); // second call should no-op
    expect((systemHealth as any).timerId).toBe(firstTimerId);
    systemHealth.stop();
  });

  it('clears timerId on stop', () => {
    systemHealth.start();
    systemHealth.stop();
    expect((systemHealth as any).timerId).toBeNull();
    expect((systemHealth as any).running).toBe(false);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Exported constants sanity check
// ─────────────────────────────────────────────────────────────────────────────

describe('exported constants', () => {
  it('FAILURE_THRESHOLD is a positive integer', () => {
    expect(Number.isInteger(FAILURE_THRESHOLD)).toBe(true);
    expect(FAILURE_THRESHOLD).toBeGreaterThan(0);
  });

  it('BACKOFF_INTERVAL_MS is greater than POLL_INTERVAL_MS', () => {
    expect(BACKOFF_INTERVAL_MS).toBeGreaterThan(POLL_INTERVAL_MS);
  });
});
