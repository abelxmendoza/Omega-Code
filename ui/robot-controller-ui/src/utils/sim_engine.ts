/**
 * SimEngine — frontend-only unicycle motion model for Demo Mode.
 *
 * Runs at ~10 Hz (DT = 0.1 s) via setInterval. Pure class, zero React
 * dependencies so it can be created once and shared via context.
 *
 * ── Behaviors ────────────────────────────────────────────────────────────────
 *   idle        No active behavior. Manual velocity passthrough.
 *   waypoint    Point-to-point nav through an ordered list, then → idle.
 *   patrol      Same as waypoint but loops back to WP 0 on completion.
 *   dock        Navigate to world origin (0, 0), then → idle.
 *   aruco_seek  Spin-to-search → align → approach nearest (or target) marker.
 *
 * ── Priority stack (highest wins each tick) ───────────────────────────────────
 *   1. Obstacle avoidance — ultrasonic gate, overrides all behaviors
 *   2. Active behavior    — aruco_seek | waypoint | patrol | dock
 *   3. Manual velocity    — only when mode === 'idle'
 *
 * ── Sensor simulation ────────────────────────────────────────────────────────
 *   Camera : FOV cone + range → MarkerDetection list (bearing/distance noise)
 *   Sonar  : 3-ray forward cast vs obstacle circles → nearest distance (noise)
 *
 *   Sensors are computed once per tick, passed to the behavior layer first
 *   (decisions), then emitted to UI subscribers (display).
 *
 * ── Path planning ─────────────────────────────────────────────────────────────
 *   A* grid planner runs once when navigation starts. Result stored in
 *   plannedPath and broadcast via telemetry — visualization only, the robot
 *   still uses its existing P-control heading correction to drive.
 */

// ── Public types ──────────────────────────────────────────────────────────────

export interface SimPose {
  x:         number;   // metres, world frame
  y:         number;
  theta_rad: number;   // radians, normalised to [-π, π]
}

export interface SimWaypoint {
  x: number;
  y: number;
}

export type BehaviorMode = 'idle' | 'waypoint' | 'aruco_seek' | 'patrol' | 'dock';

// ── World model ───────────────────────────────────────────────────────────────

export interface VirtualMarker {
  id: number;
  x:  number;
  y:  number;
}

export interface VirtualObstacle {
  x:      number;
  y:      number;
  radius: number;
}

// ── Sensor output ─────────────────────────────────────────────────────────────

export interface MarkerDetection {
  id:       number;
  distance: number;   // metres (with noise)
  bearing:  number;   // radians relative to robot heading, negative = left
  x:        number;   // world frame (no noise applied to position)
  y:        number;
}

export interface UltrasonicReading {
  distance: number | null;
}

// ── Telemetry ─────────────────────────────────────────────────────────────────

/** Single-stream snapshot emitted every tick. Replaces individual subscriptions
 *  for anything that needs a unified view of the sim state. */
export interface BehaviorTelemetry {
  mode:          BehaviorMode;
  ultraDist:     number | null;
  detections:    MarkerDetection[];
  /** A* planned path from current pose through all waypoints. Empty when idle
   *  or when no obstacles require replanning. */
  plannedPath:   SimWaypoint[];
  /** Occupancy grid: key = "cx,cy" grid integers, value = confidence [0–1]. */
  occupancyGrid: Map<string, number>;
}

// ── Behavior configuration ────────────────────────────────────────────────────

export interface BehaviorConfig {
  // Obstacle avoidance
  avoidStopDistM:    number;   // stop + rotate when sonar reads below this
  avoidTurnRate:     number;   // rad/s CCW while blocked

  // ArUco seek
  seekKp:            number;   // P-gain: bearing error → omega
  seekHeadingThresh: number;   // rad — aligned enough to start driving
  seekForwardSpeed:  number;   // m/s while approaching
  seekStopDist:      number;   // m — goal reached
  seekSearchRate:    number;   // rad/s CCW spin while no marker visible
  seekTargetId:      number | null;  // specific marker ID, or null = nearest

  // Sensor noise (realistic sim imperfection)
  bearingNoise:      number;   // rad half-range: uniform ±n
  distanceNoise:     number;   // m   half-range: uniform ±n
}

const DEFAULT_BEHAVIOR_CONFIG: BehaviorConfig = {
  avoidStopDistM:    0.25,
  avoidTurnRate:     0.8,

  seekKp:            2.5,
  seekHeadingThresh: 0.10,
  seekForwardSpeed:  0.25,
  seekStopDist:      0.35,
  seekSearchRate:    0.5,
  seekTargetId:      null,

  bearingNoise:      0.05,
  distanceNoise:     0.02,
};

// ── Listener types ────────────────────────────────────────────────────────────

export type PoseListener             = (pose: SimPose) => void;
export type WpIndexListener          = (index: number) => void;
export type MarkerDetectionsListener = (detections: MarkerDetection[]) => void;
export type UltrasonicListener       = (reading: UltrasonicReading) => void;
export type TelemetryListener        = (t: BehaviorTelemetry) => void;

// ── Physics constants ─────────────────────────────────────────────────────────

const DT              = 0.1;
const OMEGA_MAX       = 1.8;
const OMEGA_MOVE_MAX  = 0.9;
const ROTATION_KP     = 8.0;
const V_NAV           = 0.4;
const DIST_THRESH     = 0.12;
const ANGLE_THRESH    = 0.04;
const HEADING_KP      = 2.5;
const REROTATE_THRESH = 0.5;

// Speed taper: below this distance, scale velocity down to avoid overshoot.
const APPROACH_TAPER_M = 0.35;

// ── Sensor constants ──────────────────────────────────────────────────────────

const DEG                 = Math.PI / 180;
const CAMERA_FOV_HALF     = 50  * DEG;
const CAMERA_RANGE_M      = 3.0;
const ULTRASONIC_RANGE_M  = 4.0;
const ULTRASONIC_FOV_HALF = 15  * DEG;

// ── Path planning constants ───────────────────────────────────────────────────

const PLAN_CELL_M    = 0.10;   // A* grid resolution (metres per cell)
const PLAN_CLEARANCE = 0.15;   // obstacle inflation radius (metres)

// ── Occupancy grid constants ──────────────────────────────────────────────────

const OMAP_CELL_M   = 0.10;   // grid resolution (metres per cell)
const OMAP_HIT_INC  = 0.20;   // confidence added per sonar hit
const OMAP_FREE_DEC = 0.04;   // confidence removed per free-space sample

// ── Utility ───────────────────────────────────────────────────────────────────

function wrap(a: number): number {
  while (a >  Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function jitter(halfRange: number): number {
  return (Math.random() - 0.5) * 2 * halfRange;
}

function rayCircleDistance(
  ox: number, oy: number, rayAngle: number,
  cx: number, cy: number, radius: number,
): number {
  const dx  = Math.cos(rayAngle);
  const dy  = Math.sin(rayAngle);
  const fx  = cx - ox;
  const fy  = cy - oy;
  const tca = fx * dx + fy * dy;
  if (tca < 0) return Infinity;
  const d2  = fx * fx + fy * fy - tca * tca;
  const r2  = radius * radius;
  if (d2 > r2) return Infinity;
  const t = tca - Math.sqrt(r2 - d2);
  return t < 0 ? Infinity : t;
}

// ── A* path planner ───────────────────────────────────────────────────────────

/**
 * Grid-based A* over a flat 2D world.
 * Returns world-frame waypoints from `start` to `goal`, routed around obstacles.
 * Falls back to a direct segment when no obstacles are present or no path found.
 */
function planAstar(
  start:     SimWaypoint,
  goal:      SimWaypoint,
  obstacles: VirtualObstacle[],
): SimWaypoint[] {
  // Trivially clear — skip grid work
  if (obstacles.length === 0) return [goal];

  // Grid bounds: 1 m pad around the start+goal bounding box
  const pad  = 1.0;
  const minX = Math.min(start.x, goal.x) - pad;
  const minY = Math.min(start.y, goal.y) - pad;
  const maxX = Math.max(start.x, goal.x) + pad;
  const maxY = Math.max(start.y, goal.y) + pad;

  const cols = Math.ceil((maxX - minX) / PLAN_CELL_M) + 1;
  const rows = Math.ceil((maxY - minY) / PLAN_CELL_M) + 1;

  const toG = (wx: number, wy: number): [number, number] => [
    Math.round((wx - minX) / PLAN_CELL_M),
    Math.round((wy - minY) / PLAN_CELL_M),
  ];
  const toW = (gx: number, gy: number): SimWaypoint => ({
    x: minX + gx * PLAN_CELL_M,
    y: minY + gy * PLAN_CELL_M,
  });
  const cellIdx = (gx: number, gy: number) => gy * cols + gx;

  // Precompute blocked cells (obstacle footprint + robot clearance)
  const total   = cols * rows;
  const blocked = new Uint8Array(total);
  for (let gy = 0; gy < rows; gy++) {
    for (let gx = 0; gx < cols; gx++) {
      const wx = minX + gx * PLAN_CELL_M;
      const wy = minY + gy * PLAN_CELL_M;
      for (const obs of obstacles) {
        if (Math.hypot(wx - obs.x, wy - obs.y) < obs.radius + PLAN_CLEARANCE) {
          blocked[cellIdx(gx, gy)] = 1;
          break;
        }
      }
    }
  }

  const [sGx, sGy] = toG(start.x, start.y);
  const [eGx, eGy] = toG(goal.x,  goal.y);
  const startCI    = cellIdx(sGx, sGy);
  const goalCI     = cellIdx(eGx, eGy);

  const INF    = Infinity;
  const gCost  = new Float32Array(total).fill(INF);
  const parent = new Int32Array(total).fill(-1);
  const closed = new Uint8Array(total);

  gCost[startCI] = 0;

  // Heuristic: scaled Euclidean distance in grid units
  const h = (gx: number, gy: number) => Math.hypot(gx - eGx, gy - eGy);

  // Open set as a simple sorted array — fast enough for grids under 10k cells
  type ONode = { f: number; gx: number; gy: number };
  const open: ONode[] = [{ f: h(sGx, sGy), gx: sGx, gy: sGy }];

  // 8-directional movement
  const DIRS: [number, number, number][] = [
    [1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
    [1, 1, Math.SQRT2], [1, -1, Math.SQRT2],
    [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
  ];

  while (open.length > 0) {
    open.sort((a, b) => a.f - b.f);
    const cur = open.shift()!;
    const { gx, gy } = cur;
    const ci = cellIdx(gx, gy);

    if (closed[ci]) continue;
    closed[ci] = 1;

    if (ci === goalCI) {
      // Reconstruct path
      const cells: SimWaypoint[] = [];
      let at = ci;
      while (at !== startCI && at !== -1) {
        cells.unshift(toW(at % cols, Math.floor(at / cols)));
        at = parent[at];
      }
      return simplifyPath(cells);
    }

    for (const [dx, dy, cost] of DIRS) {
      const nx = gx + dx;
      const ny = gy + dy;
      if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
      const ni = cellIdx(nx, ny);
      if (blocked[ni] || closed[ni]) continue;
      const ng = gCost[ci] + cost;
      if (ng < gCost[ni]) {
        gCost[ni]  = ng;
        parent[ni] = ci;
        open.push({ f: ng + h(nx, ny), gx: nx, gy: ny });
      }
    }
  }

  // No path found — return direct line to goal as fallback
  return [goal];
}

/**
 * Remove collinear intermediate points using cross-product test.
 * Keeps start and end. Reduces path complexity for cleaner visualisation.
 */
function simplifyPath(path: SimWaypoint[]): SimWaypoint[] {
  if (path.length <= 2) return path;
  const out: SimWaypoint[] = [path[0]];
  for (let i = 1; i < path.length - 1; i++) {
    const p = out[out.length - 1];
    const c = path[i];
    const n = path[i + 1];
    const cross = (c.x - p.x) * (n.y - p.y) - (c.y - p.y) * (n.x - p.x);
    if (Math.abs(cross) > 1e-4) out.push(c);
  }
  out.push(path[path.length - 1]);
  return out;
}

/**
 * Plan a complete route: from `from` through each waypoint in sequence.
 * Each leg is independently A*-planned and concatenated (junction duplicates removed).
 */
function planFullRoute(
  from:      SimWaypoint,
  waypoints: SimWaypoint[],
  obstacles: VirtualObstacle[],
): SimWaypoint[] {
  if (waypoints.length === 0) return [];
  const full: SimWaypoint[] = [];
  let prev = from;
  for (const wp of waypoints) {
    const leg = planAstar(prev, wp, obstacles);
    // Avoid duplicating the junction point between consecutive legs
    if (full.length > 0 && leg.length > 0) {
      const last = full[full.length - 1];
      const first = leg[0];
      if (Math.hypot(last.x - first.x, last.y - first.y) < PLAN_CELL_M) {
        full.push(...leg.slice(1));
      } else {
        full.push(...leg);
      }
    } else {
      full.push(...leg);
    }
    prev = wp;
  }
  return full;
}

// ── Engine ────────────────────────────────────────────────────────────────────

export class SimEngine {
  private pose: SimPose = { x: 0, y: 0, theta_rad: 0 };

  private v     = 0;
  private omega = 0;

  private manualV     = 0;
  private manualOmega = 0;

  // Behavior
  private behaviorMode:   BehaviorMode   = 'idle';
  private behaviorConfig: BehaviorConfig = { ...DEFAULT_BEHAVIOR_CONFIG };

  // Waypoint nav state (waypoint / patrol / dock share this)
  private waypoints: SimWaypoint[] = [];
  private wpIdx:     number        = -1;
  private navPhase: 'rotating' | 'moving' | 'done' = 'done';

  // ArUco seek sub-state — tracked so getDemoFsmState() can report it
  private _seekState = 'idle';

  // A* planned path (visualization only)
  private plannedPath: SimWaypoint[] = [];

  // Occupancy grid — built from sonar hits, persists across ticks
  private occupancyGrid: Map<string, number> = new Map();

  // World model
  private markers:   VirtualMarker[]   = [];
  private obstacles: VirtualObstacle[] = [];

  // Subscriptions
  private poseListeners:             Set<PoseListener>             = new Set();
  private wpIdxListeners:            Set<WpIndexListener>          = new Set();
  private markerDetectionsListeners: Set<MarkerDetectionsListener> = new Set();
  private ultrasonicListeners:       Set<UltrasonicListener>       = new Set();
  private telemetryListeners:        Set<TelemetryListener>        = new Set();

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
    this.goIdle();
  }

  reset(): void {
    this.pose = { x: 0, y: 0, theta_rad: 0 };
    this.manualV = 0;
    this.manualOmega = 0;
    this.occupancyGrid.clear();
    this.goIdle();
    this.emitPose();
    this.emitWpIdx();
  }

  // ── World model ─────────────────────────────────────────────────────────────

  setMarkers(markers: VirtualMarker[]): void {
    this.markers = [...markers];
  }

  setObstacles(obstacles: VirtualObstacle[]): void {
    this.obstacles = [...obstacles];
  }

  // ── Behavior configuration ──────────────────────────────────────────────────

  setBehaviorConfig(config: Partial<BehaviorConfig>): void {
    this.behaviorConfig = { ...this.behaviorConfig, ...config };
  }

  // ── Behavior commands ───────────────────────────────────────────────────────

  setVelocity(v: number, omega: number): void {
    this.manualV     = v;
    this.manualOmega = omega;
  }

  /**
   * Navigate through an ordered list of waypoints.
   * @param loop  If true, restart from WP 0 on completion (patrol).
   */
  navigateTo(waypoints: SimWaypoint[], loop = false): void {
    this.behaviorMode = loop ? 'patrol' : 'waypoint';
    this.beginNav(waypoints);
  }

  /** Spin-to-search, then approach nearest (or specific) marker. */
  startArUcoSeek(targetId: number | null = null): void {
    this.behaviorConfig = { ...this.behaviorConfig, seekTargetId: targetId };
    this.behaviorMode   = 'aruco_seek';
    this.plannedPath    = [];
    this.waypoints      = [];
    this.wpIdx          = -1;
    this.navPhase       = 'done';
    this.v              = 0;
    this.omega          = 0;
    this.emitWpIdx();
  }

  /** Navigate to world origin (0, 0). */
  dock(): void {
    this.behaviorMode = 'dock';
    this.beginNav([{ x: 0, y: 0 }]);
  }

  /** Stop all active behaviors and return to idle. */
  cancelNav(): void {
    this.goIdle();
    this.emitWpIdx();
  }

  // ── State access ────────────────────────────────────────────────────────────

  getPose(): SimPose           { return { ...this.pose }; }
  getActiveWpIndex(): number   { return this.wpIdx; }
  getBehaviorMode(): BehaviorMode { return this.behaviorMode; }
  getPlannedPath(): SimWaypoint[] { return [...this.plannedPath]; }

  /**
   * Returns a human-readable FSM state label for the live badge in the UI.
   * Combines behaviorMode + navPhase (waypoint nav) or _seekState (aruco).
   */
  getDemoFsmState(): string {
    switch (this.behaviorMode) {
      case 'idle':      return 'IDLE';
      case 'aruco_seek': return this._seekState.toUpperCase();
      case 'dock':      return this.navPhase === 'done' ? 'DOCKED' : 'DOCKING';
      case 'waypoint':
      case 'patrol':
        if (this.navPhase === 'rotating') return 'TURNING';
        if (this.navPhase === 'moving')   return 'NAVIGATING';
        return 'DONE';
      default: return this.behaviorMode.toUpperCase();
    }
  }

  isNavigating(): boolean {
    if (this.behaviorMode === 'aruco_seek') return true;
    if (this.behaviorMode === 'idle') return false;
    return this.navPhase !== 'done';
  }

  // ── Subscriptions ────────────────────────────────────────────────────────────

  subscribePose(fn: PoseListener): () => void {
    this.poseListeners.add(fn);
    fn({ ...this.pose });
    return () => this.poseListeners.delete(fn);
  }

  subscribeWpIndex(fn: WpIndexListener): () => void {
    this.wpIdxListeners.add(fn);
    fn(this.wpIdx);
    return () => this.wpIdxListeners.delete(fn);
  }

  subscribeMarkerDetections(fn: MarkerDetectionsListener): () => void {
    this.markerDetectionsListeners.add(fn);
    fn([]);
    return () => this.markerDetectionsListeners.delete(fn);
  }

  subscribeUltrasonic(fn: UltrasonicListener): () => void {
    this.ultrasonicListeners.add(fn);
    fn({ distance: null });
    return () => this.ultrasonicListeners.delete(fn);
  }

  /** Unified telemetry stream: mode + sonar + camera + planned path + occupancy, every tick. */
  subscribeTelemetry(fn: TelemetryListener): () => void {
    this.telemetryListeners.add(fn);
    fn({
      mode:          this.behaviorMode,
      ultraDist:     null,
      detections:    [],
      plannedPath:   [...this.plannedPath],
      occupancyGrid: this.occupancyGrid,
    });
    return () => this.telemetryListeners.delete(fn);
  }

  // ── Simulation step ─────────────────────────────────────────────────────────

  private tick(): void {
    // 1. Sense — compute once, used by both behavior and emit
    const ultraDist  = this.computeUltrasonicDist();
    const detections = this.computeMarkerDetections();

    // 2. Behavior update → sets this.v / this.omega
    switch (this.behaviorMode) {
      case 'idle':
        this.v     = this.manualV;
        this.omega = this.manualOmega;
        break;
      case 'waypoint':
      case 'patrol':
      case 'dock':
        this.updateWaypointNav();
        break;
      case 'aruco_seek':
        this.updateArUcoSeek(detections);
        break;
    }

    // 3. Obstacle override — highest priority, always wins
    if (ultraDist !== null && ultraDist < this.behaviorConfig.avoidStopDistM) {
      this.v     = 0;
      this.omega = this.behaviorConfig.avoidTurnRate;
    }

    // 4. Integrate
    this.integrate();

    // 5. Update occupancy grid from sonar reading
    this.updateOccupancyGrid(ultraDist);

    // 6. Emit
    this.emitMarkerDetections(detections);
    this.emitUltrasonicDist(ultraDist);
    this.emitTelemetry(ultraDist, detections);
    this.emitPose();
  }

  // ── Behavior: waypoint / patrol / dock ──────────────────────────────────────

  private updateWaypointNav(): void {
    if (this.navPhase === 'done' || this.wpIdx < 0 || this.wpIdx >= this.waypoints.length) {
      this.finishNav();
      return;
    }

    const { x, y, theta_rad } = this.pose;
    const target = this.waypoints[this.wpIdx];
    const dx     = target.x - x;
    const dy     = target.y - y;
    const dist   = Math.hypot(dx, dy);

    if (this.navPhase === 'rotating') {
      const err = wrap(Math.atan2(dy, dx) - theta_rad);
      if (Math.abs(err) < ANGLE_THRESH) {
        this.omega    = 0;
        this.navPhase = 'moving';
      } else {
        this.omega = Math.max(-OMEGA_MAX, Math.min(OMEGA_MAX, err * ROTATION_KP));
        this.v     = 0;
      }
      return;
    }

    // Moving phase
    if (dist < DIST_THRESH) {
      this.v     = 0;
      this.omega = 0;
      this.wpIdx += 1;
      this.finishNav();
      return;
    }

    const headErr = wrap(Math.atan2(dy, dx) - theta_rad);

    if (Math.abs(headErr) > REROTATE_THRESH) {
      // Heading drifted too far — realign before continuing
      this.v        = 0;
      this.omega    = 0;
      this.navPhase = 'rotating';
      return;
    }

    // Taper speed as we close in — removes jitter / overshoot near the waypoint
    const speedScale = Math.min(1, dist / APPROACH_TAPER_M);
    this.v     = V_NAV * Math.max(0.35, speedScale);
    this.omega = Math.max(-OMEGA_MOVE_MAX, Math.min(OMEGA_MOVE_MAX, headErr * HEADING_KP));
  }

  /**
   * Called whenever wpIdx was just advanced.
   * Routes to: next waypoint (rotate), patrol loop, or idle.
   */
  private finishNav(): void {
    if (this.wpIdx >= 0 && this.wpIdx < this.waypoints.length) {
      // More waypoints to go — begin rotating toward the next one
      this.navPhase = 'rotating';
      this.emitWpIdx();
      return;
    }

    if (this.behaviorMode === 'patrol' && this.waypoints.length > 0) {
      // Loop: restart from WP 0
      this.wpIdx    = 0;
      this.navPhase = 'rotating';
      this.emitWpIdx();
      return;
    }

    // Route complete (waypoint / dock)
    this.goIdle();
    this.emitWpIdx();
  }

  // ── Behavior: ArUco seek ────────────────────────────────────────────────────

  private updateArUcoSeek(detections: MarkerDetection[]): void {
    const cfg = this.behaviorConfig;

    // Select target: specified ID or nearest visible marker
    const target = cfg.seekTargetId !== null
      ? (detections.find((d) => d.id === cfg.seekTargetId) ?? null)
      : detections.length > 0
        ? detections.reduce((a, b) => (a.distance < b.distance ? a : b))
        : null;

    if (target === null) {
      this._seekState = 'searching';
      this.v     = 0;
      this.omega = cfg.seekSearchRate;
      return;
    }

    if (target.distance < cfg.seekStopDist) {
      this._seekState = 'stopped';
      this.v     = 0;
      this.omega = 0;
      return;
    }

    if (Math.abs(target.bearing) > cfg.seekHeadingThresh) {
      this._seekState = 'aligning';
      this.v     = 0;
      this.omega = Math.max(-OMEGA_MAX, Math.min(OMEGA_MAX, cfg.seekKp * target.bearing));
      return;
    }

    // Aligned — drive forward. Apply bearing correction only outside a small
    // deadband so the robot doesn't oscillate when it's essentially centred.
    this._seekState = 'approaching';
    this.v = cfg.seekForwardSpeed;
    this.omega = Math.abs(target.bearing) > 0.02
      ? Math.max(-OMEGA_MOVE_MAX, Math.min(OMEGA_MOVE_MAX, cfg.seekKp * target.bearing))
      : 0;
  }

  // ── Internal helpers ────────────────────────────────────────────────────────

  /** Transition to idle: zero everything, clear nav state. */
  private goIdle(): void {
    this.behaviorMode = 'idle';
    this.waypoints    = [];
    this.wpIdx        = -1;
    this.navPhase     = 'done';
    this._seekState   = 'idle';
    this.v            = 0;
    this.omega        = 0;
  }

  /**
   * Begin navigating a waypoint list without changing behaviorMode.
   * Computes the A* planned path and emits via telemetry.
   */
  private beginNav(waypoints: SimWaypoint[]): void {
    this.waypoints = [...waypoints];

    if (waypoints.length === 0) {
      this.goIdle();
      this.emitWpIdx();
      return;
    }

    this.wpIdx    = 0;
    this.navPhase = 'rotating';

    // Plan the full route for visualisation (non-blocking — fast for small worlds)
    this.plannedPath = planFullRoute(
      { x: this.pose.x, y: this.pose.y },
      this.waypoints,
      this.obstacles,
    );

    this.emitWpIdx();
  }

  // ── Sensor computation ───────────────────────────────────────────────────────

  private computeMarkerDetections(): MarkerDetection[] {
    const { x, y, theta_rad } = this.pose;
    const cfg = this.behaviorConfig;
    const out: MarkerDetection[] = [];

    for (const m of this.markers) {
      const dx   = m.x - x;
      const dy   = m.y - y;
      const dist = Math.hypot(dx, dy);
      if (dist > CAMERA_RANGE_M) continue;

      const rawBearing = wrap(Math.atan2(dy, dx) - theta_rad);
      if (Math.abs(rawBearing) > CAMERA_FOV_HALF) continue;

      out.push({
        id:       m.id,
        distance: Math.max(0, dist       + jitter(cfg.distanceNoise)),
        bearing:  rawBearing             + jitter(cfg.bearingNoise),
        x:        m.x,
        y:        m.y,
      });
    }

    return out;
  }

  private computeUltrasonicDist(): number | null {
    const { x, y, theta_rad } = this.pose;
    if (this.obstacles.length === 0) return null;

    const rays = [
      theta_rad - ULTRASONIC_FOV_HALF,
      theta_rad,
      theta_rad + ULTRASONIC_FOV_HALF,
    ];

    let minDist = Infinity;
    for (const ra of rays) {
      for (const obs of this.obstacles) {
        const d = rayCircleDistance(x, y, ra, obs.x, obs.y, obs.radius);
        if (d < minDist) minDist = d;
      }
    }

    if (minDist > ULTRASONIC_RANGE_M) return null;
    return Math.max(0, minDist + jitter(this.behaviorConfig.distanceNoise));
  }

  // ── Occupancy grid ───────────────────────────────────────────────────────────

  private static cellKey(wx: number, wy: number): string {
    return `${Math.round(wx / OMAP_CELL_M)},${Math.round(wy / OMAP_CELL_M)}`;
  }

  private updateOccupancyGrid(ultraDist: number | null): void {
    if (ultraDist === null) return;

    const { x, y, theta_rad } = this.pose;

    // Mark hit cell
    const hitX   = x + ultraDist * Math.cos(theta_rad);
    const hitY   = y + ultraDist * Math.sin(theta_rad);
    const hitKey = SimEngine.cellKey(hitX, hitY);
    this.occupancyGrid.set(hitKey, Math.min(1, (this.occupancyGrid.get(hitKey) ?? 0) + OMAP_HIT_INC));

    // Mark free cells along the ray (low confidence reduction — optional)
    const steps = Math.floor(ultraDist / OMAP_CELL_M);
    for (let i = 1; i < steps; i++) {
      const t      = i * OMAP_CELL_M;
      const freeX  = x + t * Math.cos(theta_rad);
      const freeY  = y + t * Math.sin(theta_rad);
      const fKey   = SimEngine.cellKey(freeX, freeY);
      const cur    = this.occupancyGrid.get(fKey) ?? 0;
      // Only reduce if well below hit threshold — avoids erasing confirmed walls
      if (cur < 0.4) {
        const next = cur - OMAP_FREE_DEC;
        if (next <= 0) this.occupancyGrid.delete(fKey);
        else           this.occupancyGrid.set(fKey, next);
      }
    }
  }

  // ── Integration ──────────────────────────────────────────────────────────────

  private integrate(): void {
    const { x, y, theta_rad } = this.pose;
    this.pose = {
      x:         x + this.v * Math.cos(theta_rad) * DT,
      y:         y + this.v * Math.sin(theta_rad) * DT,
      theta_rad: wrap(theta_rad + this.omega * DT),
    };
  }

  // ── Emitters ─────────────────────────────────────────────────────────────────

  private emitPose(): void {
    const snap = { ...this.pose };
    this.poseListeners.forEach((fn) => fn(snap));
  }

  private emitWpIdx(): void {
    this.wpIdxListeners.forEach((fn) => fn(this.wpIdx));
  }

  private emitMarkerDetections(detections: MarkerDetection[]): void {
    if (this.markerDetectionsListeners.size === 0) return;
    this.markerDetectionsListeners.forEach((fn) => fn(detections));
  }

  private emitUltrasonicDist(distance: number | null): void {
    if (this.ultrasonicListeners.size === 0) return;
    this.ultrasonicListeners.forEach((fn) => fn({ distance }));
  }

  private emitTelemetry(ultraDist: number | null, detections: MarkerDetection[]): void {
    if (this.telemetryListeners.size === 0) return;
    const t: BehaviorTelemetry = {
      mode:          this.behaviorMode,
      ultraDist,
      detections,
      plannedPath:   this.plannedPath,
      occupancyGrid: this.occupancyGrid,
    };
    this.telemetryListeners.forEach((fn) => fn(t));
  }
}
