/**
 * MissionMap — full 2D mission control canvas.
 *
 * Renders in a requestAnimationFrame loop decoupled from WebSocket updates:
 *   - WebSocket pose → stored in a ref (no React re-render per frame)
 *   - rAF loop reads ref → draws at ~60 fps
 *   - React state only used for waypoints (infrequent, needs re-render)
 *
 * World → canvas coordinate system:
 *   canvasX =  cx + (worldX - viewX) * ppm
 *   canvasY =  cy - (worldY - viewY) * ppm   (+Y is UP in world, DOWN in canvas)
 *
 * Rendered layers (back to front):
 *   1. Grid (0.5 m spacing)
 *   2. ArUco marker squares + labels
 *   3. Pose trail (fading polyline)
 *   4. Camera FOV cone
 *   5. Waypoints (numbered circles + connecting line)
 *   6. Robot (filled triangle + heading line)
 *   7. Active waypoint pulse ring
 */

'use client';

import React, {
  useCallback,
  useEffect,
  useLayoutEffect,
  useRef,
  useState,
} from 'react';
import type { PoseData } from '@/hooks/usePoseStream';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

export interface MarkerDef {
  id:    number;
  x:     number;   // world metres
  y:     number;
  alpha: number;   // facing direction (radians)
}

export interface Waypoint {
  id:    number;
  x:     number;
  y:     number;
  label: string;
}

export interface MissionMapProps {
  /** Live pose from usePoseStream (null = no fix yet) */
  pose:           PoseData | null;
  /** Known ArUco marker world positions */
  markers?:       MarkerDef[];
  /** Controlled waypoint list — parent owns this state */
  waypoints:      Waypoint[];
  onWaypointsChange: (wps: Waypoint[]) => void;
  /** Index of the currently active (in-progress) waypoint */
  activeWpIndex?: number;
  /** Canvas pixel size (default fills container) */
  width?:         number;
  height?:        number;
  /** pixels-per-metre — zoom level */
  ppm?:           number;
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const TRAIL_MAX         = 300;
const GRID_SPACING_M    = 0.5;
const FOV_HALF_RAD      = (40 * Math.PI) / 180;
const FOV_RANGE_M       = 3.0;
const ROBOT_SIZE_PX     = 10;    // half-length of robot triangle
const MARKER_SIZE_PX    = 14;
const WP_RADIUS         = 10;

// Colour palette — dark theme
const C = {
  grid:          'rgba(255,255,255,0.06)',
  gridAxis:      'rgba(255,255,255,0.18)',
  trail:         '#3b82f6',          // blue-500
  trailFade:     'rgba(59,130,246,', // open rgba — append alpha + ')'
  robot:         '#60a5fa',          // blue-400
  robotStroke:   '#93c5fd',
  fov:           'rgba(96,165,250,0.08)',
  fovStroke:     'rgba(96,165,250,0.25)',
  marker:        '#f59e0b',          // amber-500
  markerStroke:  '#fde68a',
  markerLabel:   '#fde68a',
  markerNormal:  'rgba(245,158,11,0.5)',
  wpCircle:      '#10b981',          // emerald-500
  wpActive:      '#34d399',
  wpLine:        'rgba(16,185,129,0.4)',
  wpLabel:       '#d1fae5',
  origin:        'rgba(255,255,255,0.3)',
} as const;

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------

/** World → canvas X */
function wx(worldX: number, cx: number, viewX: number, ppm: number) {
  return cx + (worldX - viewX) * ppm;
}

/** World → canvas Y (flipped: world +Y = canvas up) */
function wy(worldY: number, cy: number, viewY: number, ppm: number) {
  return cy - (worldY - viewY) * ppm;
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  W: number, H: number,
  cx: number, cy: number,
  viewX: number, viewY: number,
  ppm: number,
) {
  const stepPx = GRID_SPACING_M * ppm;
  if (stepPx < 4) return;

  // Leftmost grid line in world coords
  const startWorldX = viewX - (cx / ppm);
  const startWorldY = viewY - (cy / ppm);
  const firstGX = Math.floor(startWorldX / GRID_SPACING_M) * GRID_SPACING_M;
  const firstGY = Math.floor(startWorldY / GRID_SPACING_M) * GRID_SPACING_M;

  ctx.lineWidth = 1;

  for (let gx = firstGX; ; gx += GRID_SPACING_M) {
    const px = wx(gx, cx, viewX, ppm);
    if (px > W + stepPx) break;
    const isAxis = Math.abs(gx) < 0.001;
    ctx.strokeStyle = isAxis ? C.gridAxis : C.grid;
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke();
  }

  for (let gy = firstGY; ; gy += GRID_SPACING_M) {
    const py = wy(gy, cy, viewY, ppm);
    if (py < -stepPx) break;
    const isAxis = Math.abs(gy) < 0.001;
    ctx.strokeStyle = isAxis ? C.gridAxis : C.grid;
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke();
  }
}

function drawTrail(
  ctx: CanvasRenderingContext2D,
  trail: { x: number; y: number }[],
  cx: number, cy: number,
  viewX: number, viewY: number,
  ppm: number,
) {
  if (trail.length < 2) return;
  const n = trail.length;
  for (let i = 1; i < n; i++) {
    const alpha = 0.05 + 0.7 * (i / n);
    ctx.strokeStyle = C.trailFade + alpha.toFixed(2) + ')';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(
      wx(trail[i - 1].x, cx, viewX, ppm),
      wy(trail[i - 1].y, cy, viewY, ppm),
    );
    ctx.lineTo(
      wx(trail[i].x, cx, viewX, ppm),
      wy(trail[i].y, cy, viewY, ppm),
    );
    ctx.stroke();
  }
}

function drawFov(
  ctx: CanvasRenderingContext2D,
  robotX: number, robotY: number, theta: number,
  cx: number, cy: number,
  viewX: number, viewY: number,
  ppm: number,
) {
  const rx = wx(robotX, cx, viewX, ppm);
  const ry = wy(robotY, cy, viewY, ppm);
  const rangePx = FOV_RANGE_M * ppm;

  // Canvas theta: world +X is canvas right, world +Y is canvas up.
  // Canvas angle for robot heading: -theta (Y-axis flip).
  const canvasTheta = -theta;

  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.arc(rx, ry, rangePx, canvasTheta - FOV_HALF_RAD, canvasTheta + FOV_HALF_RAD);
  ctx.closePath();
  ctx.fillStyle = C.fov;
  ctx.fill();
  ctx.strokeStyle = C.fovStroke;
  ctx.lineWidth = 1;
  ctx.stroke();
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  robotX: number, robotY: number, theta: number,
  cx: number, cy: number,
  viewX: number, viewY: number,
  ppm: number,
) {
  const rx = wx(robotX, cx, viewX, ppm);
  const ry = wy(robotY, cy, viewY, ppm);
  const canvasTheta = -theta;    // flip Y

  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(canvasTheta);

  // Filled triangle pointing forward (+canvas-X after rotate)
  const s = ROBOT_SIZE_PX;
  ctx.beginPath();
  ctx.moveTo( s,      0);      // tip (forward)
  ctx.lineTo(-s * 0.7,  s * 0.6);  // rear-right
  ctx.lineTo(-s * 0.7, -s * 0.6);  // rear-left
  ctx.closePath();
  ctx.fillStyle   = C.robot;
  ctx.fill();
  ctx.strokeStyle = C.robotStroke;
  ctx.lineWidth   = 1.5;
  ctx.stroke();

  ctx.restore();
}

function drawMarkers(
  ctx: CanvasRenderingContext2D,
  markers: MarkerDef[],
  cx: number, cy: number,
  viewX: number, viewY: number,
  ppm: number,
) {
  for (const m of markers) {
    const mx = wx(m.x, cx, viewX, ppm);
    const my = wy(m.y, cy, viewY, ppm);

    // Square
    const half = MARKER_SIZE_PX / 2;
    ctx.fillStyle   = 'rgba(245,158,11,0.15)';
    ctx.strokeStyle = C.markerStroke;
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.rect(mx - half, my - half, MARKER_SIZE_PX, MARKER_SIZE_PX);
    ctx.fill();
    ctx.stroke();

    // Normal direction line
    const normalLen = 16;
    ctx.strokeStyle = C.markerNormal;
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(mx, my);
    ctx.lineTo(
      mx + normalLen * Math.cos(-m.alpha),   // alpha in world → flip for canvas
      my + normalLen * Math.sin(-m.alpha),
    );
    ctx.stroke();

    // ID label
    ctx.fillStyle  = C.markerLabel;
    ctx.font       = 'bold 10px monospace';
    ctx.textAlign  = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`M${m.id}`, mx, my);
  }
}

function drawWaypoints(
  ctx: CanvasRenderingContext2D,
  waypoints: Waypoint[],
  activeIdx: number,
  cx: number, cy: number,
  viewX: number, viewY: number,
  ppm: number,
  animT: number,   // monotonic time for pulse animation
) {
  if (waypoints.length === 0) return;

  // Connecting line — draw done segment in muted color, pending in full color
  if (waypoints.length > 1) {
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    waypoints.forEach((wp, i) => {
      if (i === 0) return;
      const prev = waypoints[i - 1];
      const isDoneSegment = activeIdx > 0 && i <= activeIdx;
      ctx.strokeStyle = isDoneSegment
        ? 'rgba(107,114,128,0.35)'   // gray-500 dimmed
        : C.wpLine;
      ctx.beginPath();
      ctx.moveTo(wx(prev.x, cx, viewX, ppm), wy(prev.y, cy, viewY, ppm));
      ctx.lineTo(wx(wp.x,   cx, viewX, ppm), wy(wp.y,   cy, viewY, ppm));
      ctx.stroke();
    });
    ctx.setLineDash([]);
  }

  waypoints.forEach((wp, i) => {
    const px = wx(wp.x, cx, viewX, ppm);
    const py = wy(wp.y, cy, viewY, ppm);
    const isDone   = activeIdx >= 0 && i < activeIdx;
    const isActive = i === activeIdx;

    // Pulse ring for active waypoint
    if (isActive) {
      const pulseR = WP_RADIUS + 6 + 4 * Math.sin(animT * 3);
      ctx.beginPath();
      ctx.arc(px, py, pulseR, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(52,211,153,0.4)';
      ctx.lineWidth   = 1.5;
      ctx.stroke();
    }

    // Circle fill — three visual states
    ctx.beginPath();
    ctx.arc(px, py, WP_RADIUS, 0, Math.PI * 2);
    if (isDone) {
      ctx.fillStyle   = 'rgba(75,85,99,0.7)';    // gray-600 translucent
      ctx.fill();
      ctx.strokeStyle = '#6b7280';
    } else if (isActive) {
      ctx.fillStyle   = C.wpActive;
      ctx.fill();
      ctx.strokeStyle = '#d1fae5';
    } else {
      ctx.fillStyle   = C.wpCircle;
      ctx.fill();
      ctx.strokeStyle = '#d1fae5';
    }
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Inner label: checkmark for done, number for others
    if (isDone) {
      ctx.fillStyle    = '#9ca3af';   // gray-400
      ctx.font         = 'bold 10px monospace';
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('✓', px, py);
    } else {
      ctx.fillStyle    = isDone ? '#9ca3af' : '#052e16';
      ctx.font         = 'bold 10px monospace';
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), px, py);
    }

    // Label above
    ctx.fillStyle    = isDone ? '#6b7280' : C.wpLabel;
    ctx.font         = '9px monospace';
    ctx.textBaseline = 'bottom';
    ctx.fillText(wp.label || `WP${i + 1}`, px, py - WP_RADIUS - 3);
  });
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

const MissionMap: React.FC<MissionMapProps> = ({
  pose,
  markers       = [],
  waypoints,
  onWaypointsChange,
  activeWpIndex = -1,
  width         = 600,
  height        = 600,
  ppm           = 80,
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Legend visibility — visible by default, hides on first canvas interaction
  const [legendVisible, setLegendVisible] = useState(true);

  // Pose trail — stored in a ref so rAF can read without React state
  const trailRef  = useRef<{ x: number; y: number }[]>([]);
  // Latest pose ref — avoids closure capture issues in rAF callback
  const poseRef   = useRef<PoseData | null>(null);
  // Viewport centre (world coords that map to canvas centre)
  const viewRef   = useRef({ x: 0, y: 0 });
  // Follow mode: keep robot centred
  const followRef = useRef(true);
  // Animation frame handle
  const rafRef    = useRef<number>(0);
  // Stable refs for props used inside rAF
  const markersRef    = useRef(markers);
  const waypointsRef  = useRef(waypoints);
  const activeIdxRef  = useRef(activeWpIndex);
  const ppmRef        = useRef(ppm);
  const widthRef      = useRef(width);
  const heightRef     = useRef(height);

  // Sync mutable refs whenever props change
  useEffect(() => { markersRef.current   = markers;       }, [markers]);
  useEffect(() => { waypointsRef.current = waypoints;     }, [waypoints]);
  useEffect(() => { activeIdxRef.current = activeWpIndex; }, [activeWpIndex]);
  useEffect(() => { ppmRef.current       = ppm;           }, [ppm]);
  useEffect(() => { widthRef.current     = width;         }, [width]);
  useEffect(() => { heightRef.current    = height;        }, [height]);

  // Sync pose → ref + update trail
  useEffect(() => {
    if (!pose) return;
    poseRef.current = pose;

    const trail = trailRef.current;
    const last  = trail[trail.length - 1];
    const dx    = last ? Math.abs(pose.x - last.x) : Infinity;
    const dy    = last ? Math.abs(pose.y - last.y) : Infinity;
    // Only append when robot moves ≥1 cm to avoid trail explosion while stationary
    if (dx > 0.01 || dy > 0.01) {
      trail.push({ x: pose.x, y: pose.y });
      if (trail.length > TRAIL_MAX) trail.shift();
    }
  }, [pose]);

  // ---------------------------------------------------------------------------
  // rAF draw loop — runs independently of React renders
  // ---------------------------------------------------------------------------

  const drawLoop = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) { rafRef.current = requestAnimationFrame(drawLoop); return; }
    const ctx = canvas.getContext('2d');
    if (!ctx)  { rafRef.current = requestAnimationFrame(drawLoop); return; }

    const W   = widthRef.current;
    const H   = heightRef.current;
    const _ppm = ppmRef.current;
    const cx   = W / 2;
    const cy   = H / 2;
    const p    = poseRef.current;
    const animT = performance.now() / 1000;

    // Keep viewport centred on robot when follow mode is on
    if (followRef.current && p) {
      viewRef.current = { x: p.x, y: p.y };
    }
    const { x: viewX, y: viewY } = viewRef.current;

    ctx.clearRect(0, 0, W, H);

    // 1. Grid
    drawGrid(ctx, W, H, cx, cy, viewX, viewY, _ppm);

    // 2. Markers
    drawMarkers(ctx, markersRef.current, cx, cy, viewX, viewY, _ppm);

    // 3. Trail
    drawTrail(ctx, trailRef.current, cx, cy, viewX, viewY, _ppm);

    // 4. FOV cone
    if (p) {
      drawFov(ctx, p.x, p.y, p.theta_rad, cx, cy, viewX, viewY, _ppm);
    }

    // 5. Waypoints
    drawWaypoints(ctx, waypointsRef.current, activeIdxRef.current, cx, cy, viewX, viewY, _ppm, animT);

    // 6. Robot
    if (p) {
      drawRobot(ctx, p.x, p.y, p.theta_rad, cx, cy, viewX, viewY, _ppm);
    } else {
      // "Waiting for pose" indicator at origin
      const ox = wx(0, cx, viewX, _ppm);
      const oy = wy(0, cy, viewY, _ppm);
      ctx.beginPath();
      ctx.arc(ox, oy, 8, 0, Math.PI * 2);
      ctx.fillStyle   = 'rgba(100,100,100,0.6)';
      ctx.fill();
      ctx.strokeStyle = '#6b7280';
      ctx.lineWidth   = 1;
      ctx.stroke();
    }

    // 7. Scale legend (bottom-right)
    const legendM  = 1;        // 1 m scale bar
    const legendPx = legendM * _ppm;
    const lx = W - 16 - legendPx;
    const ly = H - 14;
    ctx.strokeStyle = 'rgba(255,255,255,0.4)';
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(lx, ly); ctx.lineTo(lx + legendPx, ly);
    ctx.moveTo(lx, ly - 4); ctx.lineTo(lx, ly + 4);
    ctx.moveTo(lx + legendPx, ly - 4); ctx.lineTo(lx + legendPx, ly + 4);
    ctx.stroke();
    ctx.fillStyle    = 'rgba(255,255,255,0.35)';
    ctx.font         = '9px monospace';
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`${legendM} m`, lx + legendPx / 2, ly - 5);

    rafRef.current = requestAnimationFrame(drawLoop);
  }, []);

  // Start / stop rAF loop
  useLayoutEffect(() => {
    rafRef.current = requestAnimationFrame(drawLoop);
    return () => { cancelAnimationFrame(rafRef.current); };
  }, [drawLoop]);

  // ---------------------------------------------------------------------------
  // Click → add waypoint
  // ---------------------------------------------------------------------------

  const handleCanvasClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    // Auto-hide legend on first map interaction
    setLegendVisible(false);

    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect    = canvas.getBoundingClientRect();
    const scaleX  = canvas.width  / rect.width;
    const scaleY  = canvas.height / rect.height;
    const clickX  = (e.clientX - rect.left)  * scaleX;
    const clickY  = (e.clientY - rect.top)   * scaleY;

    const W   = widthRef.current;
    const H   = heightRef.current;
    const _ppm = ppmRef.current;
    const { x: viewX, y: viewY } = viewRef.current;

    // Invert the world→canvas transform
    const worldX = viewX + (clickX - W / 2) / _ppm;
    const worldY = viewY - (clickY - H / 2) / _ppm;

    // Check if click is near an existing waypoint → remove it
    const existingIdx = waypointsRef.current.findIndex((wp) => {
      const dx = wx(wp.x, W / 2, viewX, _ppm) - clickX;
      const dy = wy(wp.y, H / 2, viewY, _ppm) - clickY;
      return Math.sqrt(dx * dx + dy * dy) < WP_RADIUS + 4;
    });

    if (existingIdx >= 0) {
      // Remove that waypoint
      const next = waypointsRef.current.filter((_, i) => i !== existingIdx);
      onWaypointsChange(next);
      return;
    }

    // Add new waypoint
    const newId = Date.now();
    const next: Waypoint[] = [
      ...waypointsRef.current,
      {
        id:    newId,
        x:     Math.round(worldX * 1000) / 1000,
        y:     Math.round(worldY * 1000) / 1000,
        label: `WP${waypointsRef.current.length + 1}`,
      },
    ];
    onWaypointsChange(next);
  }, [onWaypointsChange]);

  // Double-click → toggle follow mode / reset view to origin
  const handleDoubleClick = useCallback(() => {
    followRef.current = !followRef.current;
    if (!followRef.current) {
      viewRef.current = { x: 0, y: 0 };
    }
  }, []);

  return (
    <div className="relative flex-shrink-0" style={{ width, height }}>
      <canvas
        ref={canvasRef}
        width={width}
        height={height}
        className="rounded-lg bg-gray-950 border border-gray-700 cursor-crosshair"
        style={{ width, height, display: 'block' }}
        onClick={handleCanvasClick}
        onDoubleClick={handleDoubleClick}
      />

      {/* No-pose overlay */}
      {!pose && (
        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
          <p className="text-gray-500 text-xs bg-gray-900/80 rounded px-3 py-2 text-center leading-snug">
            Waiting for robot position —<br />
            load a simulation or connect hardware
          </p>
        </div>
      )}

      {/* Waypoints empty hint */}
      {pose && waypoints.length === 0 && (
        <div className="absolute bottom-10 left-0 right-0 flex justify-center pointer-events-none">
          <p className="text-gray-500 text-[11px] bg-gray-900/80 rounded px-3 py-1">
            Click anywhere on the map to set a navigation goal
          </p>
        </div>
      )}

      {/* Follow mode indicator (top-left) */}
      <div className="absolute top-2 left-2 flex items-center gap-1 text-[10px] text-gray-400 bg-gray-900/70 rounded px-2 py-1 pointer-events-none">
        <span className={followRef.current ? 'text-blue-400' : 'text-gray-500'}>
          {followRef.current ? '⊙ Following' : '⊙ Free view'}
        </span>
      </div>

      {/* Legend toggle button (top-right) */}
      <button
        onClick={(e) => { e.stopPropagation(); setLegendVisible((v) => !v); }}
        className="absolute top-2 right-2 w-6 h-6 rounded-full bg-gray-800/80 border border-gray-600/50 text-gray-400 hover:text-white text-xs flex items-center justify-center"
        title="Toggle legend"
      >
        ?
      </button>

      {/* Collapsible legend (top-right, below toggle) */}
      {legendVisible && (
        <div className="absolute top-10 right-2 bg-gray-900/90 border border-gray-700/60 rounded-lg px-3 py-2 text-[11px] text-gray-300 flex flex-col gap-1.5 pointer-events-none min-w-[148px]">
          <div className="text-gray-500 text-[10px] uppercase tracking-wide mb-0.5">Legend</div>
          <div className="flex items-center gap-2">
            <span className="text-blue-400 text-base leading-none">▲</span>
            <span>Robot (heading = tip)</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="text-blue-400/60 text-base leading-none">~</span>
            <span>Path trail</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="text-amber-400 text-base leading-none">■</span>
            <span>ArUco marker</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="text-emerald-400 text-base leading-none">●</span>
            <span>Waypoint</span>
          </div>
          <div className="border-t border-gray-700 mt-0.5 pt-1 text-gray-500 text-[10px] leading-snug">
            Click map to add waypoint<br />
            Click waypoint to remove<br />
            Double-click to toggle follow
          </div>
        </div>
      )}

      {/* Bottom hint */}
      <div className="absolute bottom-2 left-2 text-[10px] text-gray-500 pointer-events-none">
        Click to add/remove waypoint · Double-click to toggle follow
      </div>
    </div>
  );
};

export default MissionMap;
