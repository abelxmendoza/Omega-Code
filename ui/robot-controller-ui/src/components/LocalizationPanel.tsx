/*
 * LocalizationPanel — SE(2) EKF pose visualization.
 *
 * Polls GET /localization/pose every POLL_MS ms (2 s) and shows:
 *   - Robot x/y position in metres (relative to where filter was last reset)
 *   - Heading θ in degrees
 *   - EKF quality score (0–1) as a colour-coded badge
 *   - ArUco correction count and last marker ID seen
 *   - Mini 2D top-down canvas: robot dot + heading arrow + uncertainty ellipse
 *
 * Rate-limit resilience:
 *   - HTTP 429 → backs off to BACKOFF_MS (30 s), shows "Rate Limited" state
 *   - HTTP error / network failure → shows "Offline" state
 *   - Backoff resets to POLL_MS on next success
 *
 * The canvas coordinate system:
 *   - Origin (0, 0) maps to canvas centre
 *   - +X maps to canvas right, +Y maps to canvas up (standard math, NOT screen)
 *   - Scale: 1 metre = PIXELS_PER_METRE canvas pixels
 */

'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Navigation, Target, Wifi } from 'lucide-react';
import { usePoseStream } from '@/hooks/usePoseStream';

// ── Constants ──────────────────────────────────────────────────────────────────

const CANVAS_SIZE       = 180;    // px (square)
const PIXELS_PER_METRE  = 40;     // 1m = 40px  →  canvas shows ±2.25 m viewport

// ── Types ──────────────────────────────────────────────────────────────────────

interface PoseData {
  x:                number;
  y:                number;
  theta_rad:        number;
  theta_deg:        number;
  covariance:       number[][];
  quality:          number;
  correction_count: number;
  last_marker_seen: number | null;
  ts:               number;
}

// ── Helpers ────────────────────────────────────────────────────────────────────

function qualityColor(q: number): string {
  if (q >= 0.7) return 'text-emerald-400';
  if (q >= 0.35) return 'text-amber-400';
  return 'text-red-400';
}

function qualityBg(q: number): string {
  if (q >= 0.7) return 'bg-emerald-900/60 border-emerald-600/50';
  if (q >= 0.35) return 'bg-amber-900/60 border-amber-600/50';
  return 'bg-red-900/60 border-red-600/50';
}

function qualityLabel(q: number): string {
  if (q >= 0.7) return 'Good';
  if (q >= 0.35) return 'Fair';
  if (q > 0) return 'Poor';
  return 'Dead';
}

function qualityDescription(q: number): string {
  if (q >= 0.7) return 'Position continuously corrected by visual markers';
  if (q >= 0.35) return 'Drifting — point camera at a marker';
  if (q > 0) return 'No recent fix — move closer to a marker';
  return 'No visual correction — using dead reckoning only';
}

// ── Canvas drawing ─────────────────────────────────────────────────────────────

function drawPose(
  ctx:  CanvasRenderingContext2D,
  pose: PoseData,
  size: number,
  ppm:  number,
) {
  ctx.clearRect(0, 0, size, size);

  const cx = size / 2;
  const cy = size / 2;

  // ── Grid ────────────────────────────────────────────────────────────────────
  ctx.strokeStyle = 'rgba(255,255,255,0.07)';
  ctx.lineWidth   = 1;
  const step = ppm * 0.5;  // grid line every 0.5 m
  for (let g = cx % step; g < size; g += step) {
    ctx.beginPath(); ctx.moveTo(g, 0); ctx.lineTo(g, size); ctx.stroke();
  }
  for (let g = cy % step; g < size; g += step) {
    ctx.beginPath(); ctx.moveTo(0, g); ctx.lineTo(size, g); ctx.stroke();
  }

  // ── Origin crosshair ────────────────────────────────────────────────────────
  ctx.strokeStyle = 'rgba(255,255,255,0.25)';
  ctx.lineWidth   = 1;
  ctx.beginPath(); ctx.moveTo(cx - 8, cy); ctx.lineTo(cx + 8, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, cy - 8); ctx.lineTo(cx, cy + 8); ctx.stroke();

  // World → canvas:  canvasX = cx + x*ppm,  canvasY = cy - y*ppm  (+Y up)
  const rx = cx + pose.x * ppm;
  const ry = cy - pose.y * ppm;

  // ── Uncertainty ellipse (from covariance diagonal σx, σy) ───────────────────
  const sx = Math.sqrt(Math.abs(pose.covariance[0][0])) * ppm;
  const sy = Math.sqrt(Math.abs(pose.covariance[1][1])) * ppm;
  if (sx > 1 || sy > 1) {
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-pose.theta_rad);   // align ellipse with robot heading
    ctx.beginPath();
    ctx.ellipse(0, 0, Math.min(sx, size / 2), Math.min(sy, size / 2), 0, 0, Math.PI * 2);
    ctx.fillStyle   = 'rgba(251,191,36,0.08)';
    ctx.strokeStyle = 'rgba(251,191,36,0.30)';
    ctx.lineWidth   = 1;
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  // ── Heading arrow ───────────────────────────────────────────────────────────
  const arrowLen = 18;
  const arrowHx  = rx + arrowLen * Math.cos(-pose.theta_rad);   // -θ because canvas Y is flipped
  const arrowHy  = ry + arrowLen * Math.sin(-pose.theta_rad);
  ctx.strokeStyle = '#60a5fa';
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(arrowHx, arrowHy);
  ctx.stroke();

  // arrowhead
  const headAngle = Math.atan2(arrowHy - ry, arrowHx - rx);
  const headLen   = 7;
  ctx.beginPath();
  ctx.moveTo(arrowHx, arrowHy);
  ctx.lineTo(
    arrowHx - headLen * Math.cos(headAngle - 0.4),
    arrowHy - headLen * Math.sin(headAngle - 0.4),
  );
  ctx.moveTo(arrowHx, arrowHy);
  ctx.lineTo(
    arrowHx - headLen * Math.cos(headAngle + 0.4),
    arrowHy - headLen * Math.sin(headAngle + 0.4),
  );
  ctx.stroke();

  // ── Robot dot ───────────────────────────────────────────────────────────────
  ctx.beginPath();
  ctx.arc(rx, ry, 5, 0, Math.PI * 2);
  ctx.fillStyle   = '#60a5fa';
  ctx.fill();
  ctx.strokeStyle = '#93c5fd';
  ctx.lineWidth   = 1.5;
  ctx.stroke();
}

// ── Component ──────────────────────────────────────────────────────────────────

const LocalizationPanel: React.FC = () => {
  // ── WebSocket pose stream (replaces 2s polling) ────────────────────────────
  const { pose: wsPose, status, correctionCount, lastMarkerSeen } = usePoseStream();

  // Build a full PoseData from the stream (covariance not sent over WS — keep a
  // fallback so the uncertainty ellipse still renders from the last known value)
  const [covariance, setCovariance] = useState<number[][]>([[1,0,0],[0,1,0],[0,0,0.5]]);

  // Periodically fetch covariance from REST (low frequency — only needed for ellipse)
  useEffect(() => {
    const API_BASE = (process.env.NEXT_PUBLIC_API_URL ?? '').replace(/\/$/, '') || 'http://localhost:8000';
    let active = true;
    const fetchCov = async () => {
      try {
        const res = await fetch(`${API_BASE}/localization/pose`, { cache: 'no-store' });
        if (res.ok && active) {
          const data = await res.json();
          if (data.covariance) setCovariance(data.covariance);
        }
      } catch { /* best-effort */ }
    };
    fetchCov();
    const id = setInterval(fetchCov, 5000);   // 0.2 Hz — just for ellipse
    return () => { active = false; clearInterval(id); };
  }, []);

  const pose: PoseData | null = wsPose
    ? {
        x:                wsPose.x,
        y:                wsPose.y,
        theta_rad:        wsPose.theta_rad,
        theta_deg:        wsPose.theta_deg,
        covariance,
        quality:          wsPose.quality,
        correction_count: correctionCount,
        last_marker_seen: lastMarkerSeen,
        ts:               wsPose.ts,
      }
    : null;

  const [age, setAge] = useState(0);
  const lastTsRef     = useRef<number>(0);

  useEffect(() => {
    if (wsPose) { lastTsRef.current = Date.now(); setAge(0); }
  }, [wsPose]);

  useEffect(() => {
    const id = setInterval(() => {
      if (lastTsRef.current > 0) setAge(Math.round((Date.now() - lastTsRef.current) / 1000));
    }, 1000);
    return () => clearInterval(id);
  }, []);

  const fetchStatus =
    status === 'connected'  ? 'ok' :
    status === 'error'      ? 'offline' : 'offline';

  const canvasRef = useRef<HTMLCanvasElement>(null);

  // ── Canvas redraw ─────────────────────────────────────────────────────────
  // Draw on every fetchStatus change too so the grid appears immediately on mount.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const p: PoseData = pose ?? {
      x: 0, y: 0, theta_rad: 0, theta_deg: 0,
      covariance: [[1, 0, 0], [0, 1, 0], [0, 0, 0.5]],
      quality: 0, correction_count: 0, last_marker_seen: null, ts: 0,
    };
    drawPose(ctx, p, CANVAS_SIZE, PIXELS_PER_METRE);
  }, [pose, fetchStatus]);

  // ── Render ────────────────────────────────────────────────────────────────
  const quality = pose?.quality ?? 0;

  // Use zeroed placeholder values when no live data has arrived yet so the
  // layout is always fully visible (canvas + stats) even while offline.
  const displayPose: PoseData = pose ?? {
    x: 0, y: 0, theta_rad: 0, theta_deg: 0,
    covariance: [[1, 0, 0], [0, 1, 0], [0, 0, 0.5]],
    quality: 0, correction_count: 0, last_marker_seen: null, ts: 0,
  };

  return (
    <div className="bg-gray-800 text-white p-3 rounded-md shadow-md w-full flex flex-col gap-2">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Navigation size={14} className="text-blue-400" />
          <div>
            <h2 className="text-sm font-bold tracking-wide">Robot Position</h2>
            <p className="text-[10px] text-gray-500 leading-none">Estimated from visual markers</p>
          </div>
        </div>
        <div className={`flex items-center gap-1 px-2 py-0.5 rounded border text-xs font-medium ${qualityBg(quality)} ${qualityColor(quality)}`}
             title={qualityDescription(quality)}>
          <Target size={10} />
          <span>{qualityLabel(quality)}</span>
          {pose && <span className="ml-1 opacity-70">({(quality * 100).toFixed(0)}%)</span>}
        </div>
      </div>

      {fetchStatus === 'offline' && (
        <div className="flex items-center gap-2 text-xs text-red-400 bg-red-900/20 rounded px-2 py-2">
          <Wifi size={12} />
          <span>
            {status === 'connecting' ? 'Connecting to /ws/pose…' : 'Localization offline — backend unreachable'}
          </span>
        </div>
      )}

      {/* Stats + canvas — always rendered; shows zeroed placeholders when offline */}
      <div className="flex gap-3">
        {/* Stats column */}
        <div className="flex flex-col gap-1 min-w-[110px] text-xs">
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">X</span>
            <span className={`font-mono ${pose ? 'text-white' : 'text-gray-600'}`}>
              {displayPose.x.toFixed(3)} m
            </span>
          </div>
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">Y</span>
            <span className={`font-mono ${pose ? 'text-white' : 'text-gray-600'}`}>
              {displayPose.y.toFixed(3)} m
            </span>
          </div>
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">θ</span>
            <span className={`font-mono ${pose ? 'text-white' : 'text-gray-600'}`}>
              {displayPose.theta_deg.toFixed(1)}°
            </span>
          </div>
          <div className="border-t border-gray-700 my-0.5" />
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">Fixes</span>
            <span className={`font-mono ${displayPose.correction_count > 0 ? 'text-emerald-400' : 'text-gray-500'}`}>
              {displayPose.correction_count}
            </span>
          </div>
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">Marker</span>
            <span className="font-mono text-amber-300">
              {displayPose.last_marker_seen != null ? `#${displayPose.last_marker_seen}` : '—'}
            </span>
          </div>
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">Age</span>
            <span className={`font-mono ${pose && age > 5 ? 'text-red-400' : 'text-gray-300'}`}>
              {pose ? `${age}s` : '—'}
            </span>
          </div>
          {/* σ trace */}
          <div className="flex justify-between gap-2">
            <span className="text-gray-400">σ</span>
            <span className="font-mono text-gray-400 text-[10px]">
              {(Math.sqrt(displayPose.covariance[0][0]) * 100).toFixed(1)}cm
            </span>
          </div>
        </div>

        {/* Canvas */}
        <div className="relative flex-shrink-0">
          <canvas
            ref={canvasRef}
            width={CANVAS_SIZE}
            height={CANVAS_SIZE}
            className="rounded bg-gray-900 border border-gray-700"
            style={{ width: CANVAS_SIZE, height: CANVAS_SIZE }}
          />
          {!pose && (
            <div className="absolute inset-0 flex items-center justify-center text-gray-500 text-xs">
              Waiting…
            </div>
          )}
          {/* Scale legend */}
          <div className="absolute bottom-1 right-1 text-[9px] text-gray-600">
            {(CANVAS_SIZE / 2 / PIXELS_PER_METRE).toFixed(1)} m
          </div>
        </div>
      </div>

      {/* Quality description */}
      <p className={`text-[10px] leading-tight ${qualityColor(quality)} opacity-80`}>
        {qualityDescription(quality)}
      </p>

      {/* Hint + bridge button */}
      {(pose?.correction_count ?? 0) === 0 && (
        <p className="text-[10px] text-gray-500 leading-tight">
          No hardware? Load a simulation scenario in Mission Control.
        </p>
      )}

      <a
        href="/mission"
        target="_blank"
        rel="noopener noreferrer"
        className="mt-1 flex items-center justify-center gap-1 rounded border border-emerald-600/50 bg-emerald-900/30 px-2 py-1 text-xs font-medium text-emerald-300 hover:bg-emerald-900/50 transition-colors"
      >
        Open Mission Map ↗
      </a>
    </div>
  );
};

export default LocalizationPanel;
