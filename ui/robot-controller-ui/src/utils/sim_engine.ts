/**
 * SimEngine — frontend-only unicycle motion model for Demo Mode.
 *
 * Runs at ~10 Hz (DT = 0.1 s) via setInterval. Pure class, zero React
 * dependencies so it can be created once and shared via context.
 *
 * Waypoint navigation: rotate-in-place → drive → repeat → idle.
 * External code (context or UI) can also call setVelocity() to drive
 * manually while navPhase === 'idle'.
 */

export interface SimPose {
  x:         number;   // metres, world frame
  y:         number;   // metres, world frame
  theta_rad: number;   // radians, normalised to [-π, π]
}

export interface SimWaypoint {
  x: number;
  y: number;
}

export type PoseListener    = (pose: SimPose) => void;
export type WpIndexListener = (index: number) => void;

// ── Physics constants ─────────────────────────────────────────────────────────

const DT             = 0.1;   // integration step, seconds (10 Hz)
const OMEGA_MAX      = 1.8;   // max angular speed during rotate phase, rad/s
const OMEGA_MOVE_MAX = 0.9;   // cap on steering correction while moving
const V_NAV          = 0.4;   // forward speed during nav, m/s
const DIST_THRESH    = 0.12;  // waypoint reached when dist < this, metres
const ANGLE_THRESH   = 0.04;  // aligned enough to start driving, radians
const HEADING_KP     = 2.5;   // proportional gain for in-motion heading correction
const REROTATE_THRESH = 0.5;  // if heading drifts > ~29° while moving, stop and re-rotate

// ── Utility ───────────────────────────────────────────────────────────────────

function wrap(a: number): number {
  while (a >  Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

// ── Engine ────────────────────────────────────────────────────────────────────

export class SimEngine {
  private pose: SimPose = { x: 0, y: 0, theta_rad: 0 };

  // Manual velocity (ignored during autonomous navigation)
  private manualV     = 0;
  private manualOmega = 0;

  // Effective velocity for integration (set by updateNav or manual)
  private v     = 0;
  private omega = 0;

  // Waypoint navigation state
  private waypoints: SimWaypoint[] = [];
  private wpIdx:     number        = -1;
  private navPhase: 'idle' | 'rotating' | 'moving' = 'idle';

  // Subscriptions
  private poseListeners:  Set<PoseListener>    = new Set();
  private wpIdxListeners: Set<WpIndexListener> = new Set();

  private intervalId: ReturnType<typeof setInterval> | null = null;

  // ── Lifecycle ───────────────────────────────────────────────────────────────

  start(): void {
    if (this.intervalId !== null) return;
    this.intervalId = setInterval(() => this.tick(), DT * 1000);
  }

  stop(): void {
    if (this.intervalId !== null) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
    this.v = 0;
    this.omega = 0;
    this.navPhase = 'idle';
    this.waypoints = [];
    this.wpIdx = -1;
  }

  reset(): void {
    this.pose = { x: 0, y: 0, theta_rad: 0 };
    this.manualV = 0;
    this.manualOmega = 0;
    this.v = 0;
    this.omega = 0;
    this.navPhase = 'idle';
    this.waypoints = [];
    this.wpIdx = -1;
    this.emitPose();
    this.emitWpIdx();
  }

  // ── Commands ────────────────────────────────────────────────────────────────

  /** Manual velocity override — ignored while autonomous navigation is active. */
  setVelocity(v: number, omega: number): void {
    this.manualV     = v;
    this.manualOmega = omega;
  }

  /** Start navigating through an ordered list of waypoints. */
  navigateTo(waypoints: SimWaypoint[]): void {
    this.waypoints = [...waypoints];
    if (waypoints.length === 0) {
      this.wpIdx    = -1;
      this.navPhase = 'idle';
      this.v        = 0;
      this.omega    = 0;
    } else {
      this.wpIdx    = 0;
      this.navPhase = 'rotating';
    }
    this.emitWpIdx();
  }

  cancelNav(): void {
    this.waypoints = [];
    this.wpIdx     = -1;
    this.navPhase  = 'idle';
    this.v         = 0;
    this.omega     = 0;
    this.emitWpIdx();
  }

  // ── State access ────────────────────────────────────────────────────────────

  getPose(): SimPose          { return { ...this.pose }; }
  getActiveWpIndex(): number  { return this.wpIdx; }
  isNavigating(): boolean     { return this.navPhase !== 'idle'; }

  // ── Subscriptions ────────────────────────────────────────────────────────────

  subscribePose(fn: PoseListener): () => void {
    this.poseListeners.add(fn);
    fn({ ...this.pose });               // immediate snapshot
    return () => this.poseListeners.delete(fn);
  }

  subscribeWpIndex(fn: WpIndexListener): () => void {
    this.wpIdxListeners.add(fn);
    fn(this.wpIdx);
    return () => this.wpIdxListeners.delete(fn);
  }

  // ── Simulation step ─────────────────────────────────────────────────────────

  private tick(): void {
    this.updateNav();
    this.integrate();
    this.emitPose();
  }

  private updateNav(): void {
    if (this.navPhase === 'idle') {
      // Pass-through manual velocity
      this.v     = this.manualV;
      this.omega = this.manualOmega;
      return;
    }

    if (this.wpIdx < 0 || this.wpIdx >= this.waypoints.length) {
      this.navPhase = 'idle';
      this.v        = 0;
      this.omega    = 0;
      return;
    }

    const { x, y, theta_rad } = this.pose;
    const target = this.waypoints[this.wpIdx];
    const dx     = target.x - x;
    const dy     = target.y - y;
    const dist   = Math.hypot(dx, dy);

    if (this.navPhase === 'rotating') {
      const desired = Math.atan2(dy, dx);
      const err     = wrap(desired - theta_rad);

      if (Math.abs(err) < ANGLE_THRESH) {
        this.omega    = 0;
        this.navPhase = 'moving';
      } else {
        this.omega = Math.sign(err) * OMEGA_MAX;
        this.v     = 0;
      }
    }

    if (this.navPhase === 'moving') {
      if (dist < DIST_THRESH) {
        // Waypoint reached — advance
        this.v     = 0;
        this.omega = 0;
        this.wpIdx += 1;

        if (this.wpIdx >= this.waypoints.length) {
          this.navPhase = 'idle';
          this.wpIdx    = -1;
        } else {
          this.navPhase = 'rotating';
        }
        this.emitWpIdx();
      } else {
        // Proportional heading correction while driving
        const desired = Math.atan2(dy, dx);
        const headErr = wrap(desired - theta_rad);

        if (Math.abs(headErr) > REROTATE_THRESH) {
          // Heading has drifted too far (common when dx ≈ 0 causes atan2 to
          // jump quadrants after a slight overshoot). Stop and re-align cleanly
          // rather than trying to correct at speed, which causes spiralling.
          this.v        = 0;
          this.omega    = 0;
          this.navPhase = 'rotating';
        } else {
          this.omega = Math.max(-OMEGA_MOVE_MAX, Math.min(OMEGA_MOVE_MAX, headErr * HEADING_KP));
          this.v     = V_NAV;
        }
      }
    }
  }

  private integrate(): void {
    const { x, y, theta_rad } = this.pose;
    this.pose = {
      x:         x + this.v * Math.cos(theta_rad) * DT,
      y:         y + this.v * Math.sin(theta_rad) * DT,
      theta_rad: wrap(theta_rad + this.omega * DT),
    };
  }

  private emitPose(): void {
    const snap = { ...this.pose };
    this.poseListeners.forEach((fn) => fn(snap));
  }

  private emitWpIdx(): void {
    const idx = this.wpIdx;
    this.wpIdxListeners.forEach((fn) => fn(idx));
  }
}
