// Simple pub/sub so CameraFrame can publish its status and the header can read it.
// No deps, works on both CSR/SSR (no window refs inside the bus).
export type CameraUiState =
  | 'connecting'
  | 'connected'
  | 'disconnected'
  | 'no_camera';

export interface CameraUiStatus {
  state: CameraUiState;
  pingMs: number | null;
  lastHttp?: number | null;   // last /api/video-health HTTP code if known
  playing?: boolean;          // CameraFrame is currently playing (img) vs paused
  ts: number;                 // publish timestamp
}

type Listener = (s: CameraUiStatus) => void;

class CameraStatusBus {
  private listeners = new Set<Listener>();
  private latest: CameraUiStatus = {
    state: 'connecting',
    pingMs: null,
    lastHttp: null,
    playing: true,
    ts: Date.now(),
  };

  subscribe(fn: Listener) {
    this.listeners.add(fn);
    // push current immediately
    fn(this.latest);
    return () => this.listeners.delete(fn);
  }

  publish(update: Partial<CameraUiStatus>) {
    this.latest = { ...this.latest, ...update, ts: Date.now() };
    for (const fn of this.listeners) fn(this.latest);
  }

  getSnapshot() {
    return this.latest;
  }
}

export const cameraStatusBus = new CameraStatusBus();
