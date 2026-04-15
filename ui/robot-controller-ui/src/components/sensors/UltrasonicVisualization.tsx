/*
 * File: src/components/sensors/UltrasonicVisualization.tsx
 *
 * Phosphor-green radar display for the HC-SR04 ultrasonic sensor.
 *
 * Two modes:
 *   Static  — single forward-facing beam showing live distance (existing behaviour)
 *   Sweep   — servo pans left→right, each reading plotted as a fading radar blip
 *
 * The sweep is driven by /ws/radar on the FastAPI backend which moves the pan
 * servo (PCA9685 ch8 / servo '0') and streams {type:'radar_scan', angle_deg, distance_cm}.
 *
 * Servo angle mapping (inverted mount):
 *   servo 30°  → sensor points LEFT
 *   servo 90°  → sensor points FORWARD (up on display)
 *   servo 150° → sensor points RIGHT
 */

'use client';

import React, { useEffect, useRef, useState, useCallback } from 'react';
import { Radar, Play, Square } from 'lucide-react';
import { resolveWsCandidates } from '@/utils/resolveWsUrl';

// ── Types ─────────────────────────────────────────────────────────────────────

interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

interface RadarPoint {
  angle_deg: number;   // servo angle 30–150
  distance_cm: number;
  ts: number;          // Date.now() at receipt
}

interface UltrasonicVisualizationProps {
  data: UltrasonicData;
  isConnected: boolean;
  robotHeading?: number; // unused in radar mode, kept for API compat
}

// ── Constants ─────────────────────────────────────────────────────────────────

const MAX_RANGE_CM    = 400;
const SWEEP_MIN_DEG   = 30;
const SWEEP_MAX_DEG   = 150;
const BLIP_LIFETIME   = 10_000;   // ms — blips fade to nothing after 10s
const TRAIL_LENGTH    = 14;       // number of past angles kept for beam trail
const GREEN           = '#00ff41';

// Convert servo angle (30–150°) to canvas angle in radians.
// Canvas: 0 = right, increases clockwise.
// servo 90 → forward → top of canvas → -π/2 rad
// servo 30 → left → canvas angle -π/2 - π/3 ≈ -2.62 rad
// servo 150 → right → canvas angle -π/2 + π/3 ≈ -0.52 rad
function servoToRad(servoDeg: number): number {
  return -Math.PI / 2 + (servoDeg - 90) * (Math.PI / 180);
}

// ── Component ─────────────────────────────────────────────────────────────────

const UltrasonicVisualization: React.FC<UltrasonicVisualizationProps> = ({
  data,
  isConnected,
}) => {
  // ── canvas refs ────────────────────────────────────────────────────────────
  const canvasRef  = useRef<HTMLCanvasElement>(null);
  const animRef    = useRef<number>(0);

  // ── radar data refs (read by RAF loop — no re-renders) ─────────────────────
  const radarPointsRef  = useRef<RadarPoint[]>([]);
  const beamAngleRef    = useRef<number>(90);    // current servo angle
  const beamTrailRef    = useRef<number[]>([]);  // recent servo angles
  const isSweepingRef   = useRef<boolean>(false);
  const dataRef         = useRef<UltrasonicData>(data);
  const isConnectedRef  = useRef<boolean>(isConnected);

  // Keep sensor-data refs in sync with props (direct assignment = no effect needed)
  dataRef.current        = data;
  isConnectedRef.current = isConnected;

  // ── WS refs ────────────────────────────────────────────────────────────────
  const wsRef             = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const destroyedRef      = useRef(false);

  // ── React state (drives UI controls only) ──────────────────────────────────
  const [sweeping,          setSweeping]          = useState(false);
  const [radarConnected,    setRadarConnected]    = useState(false);
  const [hardwareAvailable, setHardwareAvailable] = useState(false);
  const [sweepError,        setSweepError]        = useState<string | null>(null);
  const [stepDeg,           setStepDeg]           = useState(5);
  const [dwellMs,           setDwellMs]           = useState(150);
  const [showSettings,      setShowSettings]      = useState(false);

  // keep ref in sync so RAF loop reads latest
  useEffect(() => { isSweepingRef.current = sweeping; }, [sweeping]);

  // ── Radar WebSocket ────────────────────────────────────────────────────────
  useEffect(() => {
    destroyedRef.current = false;

    // Derive radar URL from ultrasonic env var — same host:port, different path.
    const ultraCandidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
      defaultPort: '8000',
      path: '/ws/ultrasonic',
    });
    const radarCandidates = ultraCandidates
      .map(url => {
        try {
          const u = new URL(url);
          u.pathname = '/ws/radar';
          return u.toString();
        } catch { return ''; }
      })
      .filter(Boolean);

    if (!radarCandidates.length) {
      setSweepError('No radar WS URL — check NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC');
      return;
    }

    let idx = 0;

    const connect = () => {
      if (destroyedRef.current || idx >= radarCandidates.length) return;
      const url = radarCandidates[idx];

      try {
        const ws = new WebSocket(url);
        wsRef.current = ws;

        ws.onopen = () => {
          setRadarConnected(true);
          setSweepError(null);
          idx = 0;
        };

        ws.onmessage = (ev) => {
          try {
            const msg = JSON.parse(ev.data as string);

            if (msg.type === 'welcome') {
              setHardwareAvailable(!!msg.hardware);
              return;
            }
            if (msg.type === 'radar_scan') {
              const deg  = Number(msg.angle_deg);
              const dist = msg.distance_cm != null ? Number(msg.distance_cm) : null;

              beamAngleRef.current = deg;
              beamTrailRef.current = [
                ...beamTrailRef.current.slice(-(TRAIL_LENGTH - 1)),
                deg,
              ];
              if (dist !== null && dist > 0) {
                radarPointsRef.current.push({ angle_deg: deg, distance_cm: dist, ts: Date.now() });
              }
              return;
            }
            if (msg.type === 'sweep_status') {
              setSweeping(!!msg.sweeping);
              if (!msg.sweeping) {
                // Clear trail so beam disappears
                beamTrailRef.current = [];
              }
              if (msg.error) setSweepError(msg.error);
              return;
            }
          } catch { /* ignore malformed */ }
        };

        ws.onerror = () => setRadarConnected(false);

        ws.onclose = () => {
          setRadarConnected(false);
          setSweeping(false);
          isSweepingRef.current = false;
          beamTrailRef.current  = [];
          wsRef.current = null;
          if (!destroyedRef.current) {
            if (idx < radarCandidates.length - 1) idx++;
            reconnectTimerRef.current = setTimeout(connect, 3000);
          }
        };
      } catch {
        if (!destroyedRef.current) {
          if (idx < radarCandidates.length - 1) idx++;
          reconnectTimerRef.current = setTimeout(connect, 3000);
        }
      }
    };

    connect();

    return () => {
      destroyedRef.current = true;
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      wsRef.current?.close();
      wsRef.current = null;
    };
  }, []);

  // ── Sweep controls ─────────────────────────────────────────────────────────
  const startSweep = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    radarPointsRef.current = [];
    beamTrailRef.current   = [];
    ws.send(JSON.stringify({
      command:   'start-sweep',
      step_deg:  stepDeg,
      dwell_ms:  dwellMs,
      sweep_min: SWEEP_MIN_DEG,
      sweep_max: SWEEP_MAX_DEG,
    }));
  }, [stepDeg, dwellMs]);

  const stopSweep = useCallback(() => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({ command: 'stop-sweep' }));
  }, []);

  // ── Canvas render loop ─────────────────────────────────────────────────────
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const updateSize = () => {
      const parent = canvas.parentElement;
      if (parent) {
        const { width, height } = parent.getBoundingClientRect();
        const s = Math.max(160, Math.min(400, width - 4, height - 4));
        canvas.width  = s;
        canvas.height = s;
      }
    };

    // ─── Draw one frame ─────────────────────────────────────────────────────
    const frame = () => {
      const W  = canvas.width;
      const H  = canvas.height;
      const cx = W / 2;
      const cy = H / 2;
      const R  = Math.min(cx, cy) - 22;
      const now = Date.now();

      ctx.clearRect(0, 0, W, H);

      // ── radial gradient background ────────────────────────────────────────
      const bg = ctx.createRadialGradient(cx, cy, 0, cx, cy, R);
      bg.addColorStop(0,   '#001800');
      bg.addColorStop(0.6, '#000c00');
      bg.addColorStop(1,   '#000000');
      ctx.fillStyle = bg;
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.fill();

      // ── outer ring ────────────────────────────────────────────────────────
      ctx.strokeStyle = 'rgba(0,255,65,0.4)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(cx, cy, R, 0, Math.PI * 2);
      ctx.stroke();

      // ── sweep-zone sector (subtle tint) ───────────────────────────────────
      const aL = servoToRad(SWEEP_MIN_DEG);  // upper-left limit
      const aR = servoToRad(SWEEP_MAX_DEG);  // upper-right limit
      ctx.fillStyle = 'rgba(0,255,65,0.03)';
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, R, aL, aR);
      ctx.closePath();
      ctx.fill();

      // Sweep zone boundary dashes
      ctx.save();
      ctx.setLineDash([3, 7]);
      ctx.strokeStyle = 'rgba(0,255,65,0.22)';
      ctx.lineWidth = 1;
      for (const a of [aL, aR]) {
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(a) * R, cy + Math.sin(a) * R);
        ctx.stroke();
      }
      ctx.restore();

      // ── concentric range rings (5) ────────────────────────────────────────
      const N_RINGS = 5;
      for (let i = 1; i <= N_RINGS; i++) {
        const r = (R * i) / N_RINGS;
        ctx.strokeStyle = `rgba(0,255,65,${0.06 + i * 0.025})`;
        ctx.lineWidth   = 0.7;
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();
      }

      // ── radial grid lines every 30° ───────────────────────────────────────
      for (let deg = 0; deg < 360; deg += 30) {
        const a     = (deg - 90) * (Math.PI / 180);  // 0° at top
        const isFwd = deg === 0;
        ctx.strokeStyle = isFwd ? 'rgba(0,255,65,0.35)' : 'rgba(0,255,65,0.07)';
        ctx.lineWidth   = isFwd ? 1.2 : 0.5;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(a) * R, cy + Math.sin(a) * R);
        ctx.stroke();
      }

      // ── range labels ──────────────────────────────────────────────────────
      ctx.fillStyle  = 'rgba(0,255,65,0.45)';
      ctx.font       = '8px monospace';
      ctx.textAlign  = 'left';
      for (let i = 1; i <= N_RINGS; i++) {
        const r   = (R * i) / N_RINGS;
        const lbl = `${(MAX_RANGE_CM * i) / N_RINGS | 0}`;
        ctx.fillText(lbl, cx + 4, cy - r + 10);
      }

      // ── historical blips ──────────────────────────────────────────────────
      // Prune expired points
      const alive = radarPointsRef.current.filter(p => now - p.ts < BLIP_LIFETIME);
      radarPointsRef.current = alive;

      for (const pt of alive) {
        const fresh = 1 - (now - pt.ts) / BLIP_LIFETIME;
        const a     = servoToRad(pt.angle_deg);
        const r     = Math.min(pt.distance_cm / MAX_RANGE_CM, 1) * R;
        const bx    = cx + Math.cos(a) * r;
        const by    = cy + Math.sin(a) * r;
        const glowR = 3 + fresh * 7;

        // Glow halo
        const grd = ctx.createRadialGradient(bx, by, 0, bx, by, glowR * 2.5);
        grd.addColorStop(0,   `rgba(0,255,65,${(fresh * 0.9).toFixed(2)})`);
        grd.addColorStop(0.4, `rgba(0,255,65,${(fresh * 0.35).toFixed(2)})`);
        grd.addColorStop(1,   'rgba(0,255,65,0)');
        ctx.fillStyle = grd;
        ctx.beginPath();
        ctx.arc(bx, by, glowR * 2.5, 0, Math.PI * 2);
        ctx.fill();

        // Bright core
        ctx.fillStyle = `rgba(0,255,65,${fresh.toFixed(2)})`;
        ctx.beginPath();
        ctx.arc(bx, by, Math.max(1.5, glowR * 0.35), 0, Math.PI * 2);
        ctx.fill();
      }

      // ── sweep beam + trail ────────────────────────────────────────────────
      const trail = beamTrailRef.current;
      if (isSweepingRef.current || trail.length > 0) {

        // Fading trail (older angles drawn first, dim)
        for (let i = 0; i < trail.length; i++) {
          const alpha = ((i + 1) / trail.length) * 0.28;
          const a     = servoToRad(trail[i]);
          ctx.strokeStyle = `rgba(0,255,65,${alpha.toFixed(2)})`;
          ctx.lineWidth   = 1.5;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx + Math.cos(a) * R, cy + Math.sin(a) * R);
          ctx.stroke();
        }

        // Active beam — bright gradient with glow
        const ba  = servoToRad(beamAngleRef.current);
        const bex = cx + Math.cos(ba) * R;
        const bey = cy + Math.sin(ba) * R;

        const beamGrad = ctx.createLinearGradient(cx, cy, bex, bey);
        beamGrad.addColorStop(0,    'rgba(0,255,65,0)');
        beamGrad.addColorStop(0.15, 'rgba(0,255,65,0.12)');
        beamGrad.addColorStop(1,    'rgba(0,255,65,0.95)');

        ctx.strokeStyle = beamGrad;
        ctx.lineWidth   = 2.5;
        ctx.shadowColor = GREEN;
        ctx.shadowBlur  = 14;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(bex, bey);
        ctx.stroke();
        ctx.shadowBlur = 0;

      } else {
        // ── static mode: single forward beam with current distance blip ─────
        const fwd  = -Math.PI / 2;   // straight up
        const dist = dataRef.current?.distance_cm ?? 0;
        const conn = isConnectedRef.current;

        if (conn && dist > 0) {
          const r  = Math.min(dist / MAX_RANGE_CM, 1) * R;
          const ox = cx + Math.cos(fwd) * r;
          const oy = cy + Math.sin(fwd) * r;

          const isClose = dist < 30;
          const isMed   = dist < 100;
          const colour  = isClose ? '255,60,0' : isMed ? '255,165,0' : '0,255,65';

          // Beam line
          const beamG = ctx.createLinearGradient(cx, cy, ox, oy);
          beamG.addColorStop(0, `rgba(${colour},0)`);
          beamG.addColorStop(1, `rgba(${colour},0.85)`);
          ctx.strokeStyle = beamG;
          ctx.lineWidth   = 2;
          ctx.shadowColor = `rgb(${colour})`;
          ctx.shadowBlur  = 8;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(ox, oy);
          ctx.stroke();
          ctx.shadowBlur = 0;

          // Obstacle blip
          const pulse  = Math.sin(now / 300) * 0.15 + 0.85;
          const blipR  = 4 + (isClose ? 3 : 0);
          const glowG  = ctx.createRadialGradient(ox, oy, 0, ox, oy, blipR * 3.5);
          glowG.addColorStop(0,   `rgba(${colour},${(0.9 * pulse).toFixed(2)})`);
          glowG.addColorStop(0.4, `rgba(${colour},${(0.4 * pulse).toFixed(2)})`);
          glowG.addColorStop(1,   `rgba(${colour},0)`);
          ctx.fillStyle = glowG;
          ctx.beginPath();
          ctx.arc(ox, oy, blipR * 3.5, 0, Math.PI * 2);
          ctx.fill();

          ctx.fillStyle = `rgba(${colour},${pulse.toFixed(2)})`;
          ctx.beginPath();
          ctx.arc(ox, oy, blipR, 0, Math.PI * 2);
          ctx.fill();

          // Distance tag
          ctx.fillStyle  = `rgba(${colour},0.95)`;
          ctx.font       = 'bold 11px monospace';
          ctx.textAlign  = 'center';
          ctx.fillText(`${dist.toFixed(0)}cm`, ox, oy - blipR - 8);

          // Warning bar
          if (isClose) {
            const blink = Math.sin(now / 250) > 0;
            if (blink) {
              ctx.fillStyle = `rgba(255,60,0,0.85)`;
              ctx.font      = 'bold 10px monospace';
              ctx.textAlign = 'center';
              ctx.fillText('⚠ OBSTACLE', cx, cy + R - 12);
            }
          }
        } else {
          // No signal — faint dashed forward line
          ctx.save();
          ctx.setLineDash([4, 8]);
          ctx.strokeStyle = 'rgba(0,255,65,0.15)';
          ctx.lineWidth   = 1;
          ctx.beginPath();
          ctx.moveTo(cx, cy);
          ctx.lineTo(cx, cy - R);
          ctx.stroke();
          ctx.restore();

          ctx.fillStyle  = 'rgba(0,255,65,0.3)';
          ctx.font       = '11px monospace';
          ctx.textAlign  = 'center';
          ctx.fillText('NO SIGNAL', cx, cy + R / 2);
        }
      }

      // ── centre dot (robot position) ───────────────────────────────────────
      ctx.shadowColor = GREEN;
      ctx.shadowBlur  = 16;
      ctx.fillStyle   = GREEN;
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.shadowBlur  = 0;

      ctx.strokeStyle = 'rgba(0,255,65,0.55)';
      ctx.lineWidth   = 1.2;
      ctx.beginPath();
      ctx.arc(cx, cy, 8, 0, Math.PI * 2);
      ctx.stroke();

      // ── compass labels ────────────────────────────────────────────────────
      ctx.fillStyle  = 'rgba(0,255,65,0.65)';
      ctx.font       = 'bold 9px monospace';
      ctx.textAlign  = 'center';
      ctx.fillText('FWD', cx, cy - R + 13);

      ctx.textAlign = 'right';
      ctx.fillText('60°L', cx + Math.cos(aL) * (R + 14), cy + Math.sin(aL) * (R + 14) + 4);
      ctx.textAlign = 'left';
      ctx.fillText('60°R', cx + Math.cos(aR) * (R + 14), cy + Math.sin(aR) * (R + 14) + 4);

      // ── scan ticker (only while sweeping) ────────────────────────────────
      if (isSweepingRef.current) {
        const blink = Math.sin(now / 420) > 0;
        if (blink) {
          ctx.fillStyle  = 'rgba(0,255,65,0.85)';
          ctx.font       = '9px monospace';
          ctx.textAlign  = 'center';
          const phi = beamAngleRef.current - 90;
          const dir = phi < -3 ? 'L' : phi > 3 ? 'R' : 'FWD';
          ctx.fillText(`● SCANNING ${Math.abs(phi).toFixed(0)}°${dir}`, cx, cy + R - 10);
        }
      }

      animRef.current = requestAnimationFrame(frame);
    };

    // ── bootstrap ───────────────────────────────────────────────────────────
    updateSize();
    const ro = new ResizeObserver(() => {
      updateSize();
      cancelAnimationFrame(animRef.current);
      animRef.current = requestAnimationFrame(frame);
    });
    if (canvas.parentElement) ro.observe(canvas.parentElement);
    animRef.current = requestAnimationFrame(frame);

    return () => {
      cancelAnimationFrame(animRef.current);
      ro.disconnect();
    };
  }, []); // no deps — RAF loop reads everything from refs

  // ── UI helpers ─────────────────────────────────────────────────────────────
  const distColor = () => {
    if (!isConnected) return 'text-gray-600';
    const d = data?.distance_cm ?? 0;
    if (d === 0)  return 'text-gray-600';
    if (d < 30)   return 'text-red-400';
    if (d < 100)  return 'text-yellow-400';
    return 'text-green-400';
  };

  const sweepTimeEst =
    Math.round(((SWEEP_MAX_DEG - SWEEP_MIN_DEG) / stepDeg) * dwellMs / 100) / 10;

  // ── Render ────────────────────────────────────────────────────────────────
  return (
    <div className="flex flex-col gap-2 p-2 bg-gray-950 rounded-lg select-none min-w-0">

      {/* ── Header ── */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-1.5">
          <Radar className="h-3.5 w-3.5 text-green-400" />
          <span className="text-xs font-bold text-green-400 font-mono tracking-widest">RADAR</span>
          <span
            className={`w-1.5 h-1.5 rounded-full ${radarConnected ? 'bg-green-400 animate-pulse' : 'bg-red-600'}`}
            title={radarConnected ? 'Radar WS connected' : 'Radar WS disconnected'}
          />
          {radarConnected && !hardwareAvailable && (
            <span className="text-[9px] text-amber-400 font-mono border border-amber-500/40 px-1 rounded">
              SIM
            </span>
          )}
          {sweeping && (
            <span className="text-[9px] text-green-300 font-mono border border-green-500/40 px-1 rounded animate-pulse">
              SWEEP
            </span>
          )}
        </div>
        <span className={`font-mono font-bold text-sm ${distColor()}`}>
          {isConnected && (data?.distance_cm ?? 0) > 0
            ? `${(data.distance_cm).toFixed(0)} cm`
            : '— cm'}
        </span>
      </div>

      {/* ── Canvas ── */}
      <div
        className="relative bg-black rounded overflow-hidden border border-green-500/25"
        style={{ aspectRatio: '1 / 1' }}
      >
        <canvas ref={canvasRef} className="w-full h-full block" />
        {/* CRT scanline texture overlay */}
        <div
          className="absolute inset-0 pointer-events-none"
          style={{
            background:
              'repeating-linear-gradient(0deg,rgba(0,0,0,0.04) 0px,rgba(0,0,0,0.04) 1px,transparent 1px,transparent 2px)',
          }}
        />
      </div>

      {/* ── Distance readouts ── */}
      <div className="grid grid-cols-4 gap-1">
        {[
          { lbl: 'cm',  val: (data?.distance_cm    ?? 0).toFixed(0)  },
          { lbl: 'm',   val: (data?.distance_m     ?? 0).toFixed(2)  },
          { lbl: 'in',  val: (data?.distance_inch  ?? 0).toFixed(1)  },
          { lbl: 'ft',  val: (data?.distance_feet  ?? 0).toFixed(2)  },
        ].map(({ lbl, val }) => (
          <div key={lbl} className="bg-black/50 border border-green-500/15 rounded p-1 text-center">
            <div className="text-[9px] text-gray-600 font-mono">{lbl}</div>
            <div className="text-[11px] font-mono font-bold text-green-400">{val}</div>
          </div>
        ))}
      </div>

      {/* ── Sweep controls ── */}
      <div className="flex gap-1.5 items-center">
        {sweeping ? (
          <button
            onClick={stopSweep}
            className="flex-1 flex items-center justify-center gap-1.5 py-1.5 rounded
                       bg-red-950/70 border border-red-600/50 text-red-300
                       font-mono text-[11px] font-bold
                       hover:bg-red-900/70 active:scale-95 transition-all"
          >
            <Square className="h-3 w-3 fill-current" />
            STOP SWEEP
          </button>
        ) : (
          <button
            onClick={startSweep}
            disabled={!radarConnected}
            className="flex-1 flex items-center justify-center gap-1.5 py-1.5 rounded
                       bg-green-950/60 border border-green-600/40 text-green-300
                       font-mono text-[11px] font-bold
                       hover:bg-green-900/60 active:scale-95 transition-all
                       disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <Play className="h-3 w-3 fill-current" />
            START SWEEP
          </button>
        )}

        <button
          onClick={() => setShowSettings(s => !s)}
          title="Sweep settings"
          className={`px-2.5 py-1.5 rounded border font-mono text-xs transition-all
                      ${showSettings
                        ? 'bg-green-900/50 border-green-500/50 text-green-300'
                        : 'bg-gray-900/60 border-gray-700/50 text-gray-400 hover:bg-gray-800/60'}`}
        >
          ⚙
        </button>
      </div>

      {/* ── Settings panel ── */}
      {showSettings && (
        <div className="bg-black/60 border border-green-500/15 rounded p-2.5 space-y-2.5">
          <div className="flex items-center gap-2">
            <span className="text-[10px] text-gray-500 font-mono w-[4.5rem] shrink-0">
              Step (°)
            </span>
            <input
              type="range" min={1} max={20} step={1} value={stepDeg}
              onChange={e => setStepDeg(Number(e.target.value))}
              className="flex-1 h-1 accent-green-400"
            />
            <span className="text-[10px] font-mono text-green-400 w-6 text-right shrink-0">
              {stepDeg}°
            </span>
          </div>

          <div className="flex items-center gap-2">
            <span className="text-[10px] text-gray-500 font-mono w-[4.5rem] shrink-0">
              Dwell (ms)
            </span>
            <input
              type="range" min={50} max={500} step={25} value={dwellMs}
              onChange={e => setDwellMs(Number(e.target.value))}
              className="flex-1 h-1 accent-green-400"
            />
            <span className="text-[10px] font-mono text-green-400 w-10 text-right shrink-0">
              {dwellMs}ms
            </span>
          </div>

          <div className="text-[9px] text-gray-600 font-mono">
            ≈ {sweepTimeEst}s per half-sweep &nbsp;·&nbsp; {stepDeg}°/step &nbsp;·&nbsp; ±60° range
          </div>
        </div>
      )}

      {/* ── Error ── */}
      {sweepError && (
        <div className="text-[10px] text-rose-400 font-mono bg-rose-950/30 border border-rose-600/30 rounded px-2 py-1">
          {sweepError}
        </div>
      )}
    </div>
  );
};

export default UltrasonicVisualization;
