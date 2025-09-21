/*
# File: /Omega-Code/ui/robot-controller-ui/src/utils/cameraStatusBus.ts
# Summary:
#   Tiny SSR/CSR-safe pub/sub bus so CameraFrame can publish camera UI health and
#   other components (Header, ServiceStatusBar, etc.) can read it without prop drilling.
#   - subscribe(fn): adds a listener and immediately emits the current snapshot; returns unsubscribe()
#   - publish(partial): merges fields into the latest state, stamps ts, and notifies listeners
#   - getSnapshot(): returns the most recent state (cheap read)
#
#   Notes:
#   • No window/document usage → safe during SSR.
#   • Uses Set#forEach (not for..of) to avoid needing --downlevelIteration on older TS targets.
#   • Skips notifications when there are no meaningful changes (avoids redundant re-renders).
*/

export type CameraUiState = 'connecting' | 'connected' | 'disconnected' | 'no_camera';

export interface CameraUiStatus {
  /** High-level camera UI state. */
  state: CameraUiState;
  /** Ping RTT (ms) for camera health checks, or null if unknown. */
  pingMs: number | null;
  /** Last HTTP status observed from /api/video-health (if known). */
  lastHttp?: number | null;
  /** Whether the CameraFrame is actively showing frames (vs paused). */
  playing?: boolean;
  /** Monotonic publish timestamp in ms (Date.now()). */
  ts: number;
}

type Listener = (s: CameraUiStatus) => void;

/** Shallow equality for CameraUiStatus, ignoring `ts` (which always changes). */
function shallowEqualIgnoringTs(a: CameraUiStatus, b: CameraUiStatus): boolean {
  if (a === b) return true;
  // Compare known fields except ts
  return (
    a.state === b.state &&
    a.pingMs === b.pingMs &&
    a.lastHttp === b.lastHttp &&
    a.playing === b.playing
  );
}

class CameraStatusBus {
  private listeners = new Set<Listener>();

  // Sensible defaults so subscribers render something immediately.
  private latest: CameraUiStatus = {
    state: 'connecting',
    pingMs: null,
    lastHttp: null,
    playing: true,
    ts: Date.now(),
  };

  /**
   * Subscribe to status updates. Immediately emits the current snapshot.
   * @returns unsubscribe() cleanup function
   */
  subscribe(fn: Listener): () => void {
    this.listeners.add(fn);
    // Emit current snapshot right away so UI doesn’t wait for the next publish.
    try {
      fn(this.latest);
    } catch {
      // Listener errors should not break the bus.
    }
    return () => {
      this.listeners.delete(fn);
    };
  }

  /**
   * Publish a partial update. Merges into the latest state, stamps ts, and
   * notifies listeners if anything (excluding ts) actually changed.
   */
  publish(update: Partial<CameraUiStatus>): void {
    // Merge without ts first to check for meaningful change
    const nextWithoutTs: CameraUiStatus = {
      ...this.latest,
      ...update,
      // keep previous ts in this comparison object; we’ll set a fresh ts below
      ts: this.latest.ts,
    };

    if (shallowEqualIgnoringTs(this.latest, nextWithoutTs)) {
      // Nothing meaningful changed; skip notifying listeners.
      return;
    }

    // Commit with a fresh timestamp.
    this.latest = { ...nextWithoutTs, ts: Date.now() };

    // Notify all listeners; isolate their errors.
    this.listeners.forEach((cb) => {
      try {
        cb(this.latest);
      } catch {
        /* ignore listener errors */
      }
    });
  }

  /**
   * Get the current snapshot. Returns the live object for performance; treat as readonly.
   * If you need immutability, spread it: `{ ...cameraStatusBus.getSnapshot() }`.
   */
  getSnapshot(): CameraUiStatus {
    return this.latest;
  }
}

export const cameraStatusBus = new CameraStatusBus();
