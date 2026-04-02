/*
# File: /src/services/systemHealth.ts
# Summary:
#   Singleton system health service. Single source of truth for "is the backend up?".
#
#   Polls /api/health at a stable interval. Circuit breaker:
#     - 3 consecutive failures → degraded, interval backs off to 60s
#     - Any success resets failure count and restores normal interval
#
#   Components read state via subscribe() rather than running their own health probes.
#   This eliminates the N×(polling) problem where every component independently hits
#   the backend to check if it's alive.
#
#   API:
#     systemHealth.getState()           → SystemHealthState
#     systemHealth.subscribe(callback)  → unsubscribe function
#     systemHealth.start() / .stop()    → lifecycle control (called by SystemHealthContext)
*/

export type ComponentHealth = 'ok' | 'degraded' | 'down';

export interface SystemHealthState {
  connected: boolean;
  degraded: boolean;
  lastUpdated: number;
  components: {
    backend: ComponentHealth;
    camera: ComponentHealth;
    sensors: ComponentHealth;
    performance: ComponentHealth;
  };
}

type Subscriber = (state: SystemHealthState) => void;

const POLL_INTERVAL_MS    = 15_000; // normal: every 15s
const BACKOFF_INTERVAL_MS = 60_000; // after 3 failures: every 60s
const FAILURE_THRESHOLD   = 3;
const PROBE_TIMEOUT_MS    = 4_000;

const DEFAULT_STATE: SystemHealthState = {
  connected: false,
  degraded: false,
  lastUpdated: 0,
  components: {
    backend:     'down',
    camera:      'down',
    sensors:     'down',
    performance: 'down',
  },
};

class SystemHealthService {
  private state: SystemHealthState = { ...DEFAULT_STATE };
  private subscribers = new Set<Subscriber>();
  private timerId: ReturnType<typeof setInterval> | null = null;
  private failureCount = 0;
  private activeInterval = POLL_INTERVAL_MS;
  private running = false;

  // ---- Public API --------------------------------------------------------

  getState(): SystemHealthState {
    return { ...this.state };
  }

  subscribe(cb: Subscriber): () => void {
    this.subscribers.add(cb);
    // Immediately emit current state so subscriber doesn't wait for next poll
    cb(this.getState());
    return () => this.subscribers.delete(cb);
  }

  start(): void {
    if (this.running) return;
    this.running = true;
    this.failureCount = 0;
    this.activeInterval = POLL_INTERVAL_MS;
    void this.probe();
    this.timerId = setInterval(() => void this.probe(), POLL_INTERVAL_MS);
  }

  stop(): void {
    this.running = false;
    if (this.timerId) {
      clearInterval(this.timerId);
      this.timerId = null;
    }
  }

  // ---- Internal ----------------------------------------------------------

  private notify(): void {
    const snapshot = this.getState();
    this.subscribers.forEach((cb) => {
      try { cb(snapshot); } catch { /* subscriber errors must not affect service */ }
    });
  }

  private restartTimer(intervalMs: number): void {
    if (this.timerId) clearInterval(this.timerId);
    this.activeInterval = intervalMs;
    this.timerId = setInterval(() => void this.probe(), intervalMs);
  }

  private async probe(): Promise<void> {
    if (!this.running) return;

    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), PROBE_TIMEOUT_MS);

    try {
      const res = await fetch('/api/health', {
        method: 'GET',
        cache: 'no-store',
        signal: controller.signal,
      });

      if (res.ok) {
        const wasBackedOff = this.failureCount >= FAILURE_THRESHOLD;
        this.failureCount = 0;

        this.state = {
          connected: true,
          degraded: false,
          lastUpdated: Date.now(),
          components: {
            backend:     'ok',
            camera:      this.state.components.camera,      // preserved from last known state
            sensors:     this.state.components.sensors,
            performance: this.state.components.performance,
          },
        };

        if (wasBackedOff) this.restartTimer(POLL_INTERVAL_MS); // recover from back-off
      } else {
        this.handleFailure();
      }
    } catch {
      this.handleFailure();
    } finally {
      clearTimeout(timeout);
    }

    this.notify();
  }

  private handleFailure(): void {
    this.failureCount += 1;
    const degraded = this.failureCount >= FAILURE_THRESHOLD;

    this.state = {
      connected: false,
      degraded,
      lastUpdated: Date.now(),
      components: {
        backend:     degraded ? 'down' : 'degraded',
        camera:      degraded ? 'down' : this.state.components.camera,
        sensors:     degraded ? 'down' : this.state.components.sensors,
        performance: degraded ? 'down' : this.state.components.performance,
      },
    };

    // Circuit breaker: escalate to back-off interval after threshold
    if (this.failureCount === FAILURE_THRESHOLD && this.activeInterval !== BACKOFF_INTERVAL_MS) {
      this.restartTimer(BACKOFF_INTERVAL_MS);
    }
  }
}

// Singleton — one polling loop for the entire app lifetime
export const systemHealth = new SystemHealthService();

// Export thresholds for tests
export { FAILURE_THRESHOLD, POLL_INTERVAL_MS, BACKOFF_INTERVAL_MS };
