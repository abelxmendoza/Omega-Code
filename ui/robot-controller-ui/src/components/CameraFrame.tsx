/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/CameraFrame.tsx
# Summary:
#   MJPEG camera frame with controls (play/pause, fit, rotate, flip, filters,
#   snapshot/record, fullscreen) and a prominent camera power button.
#
#   Camera power state is owned by the backend (CameraPowerManager singleton).
#   The UI fetches authoritative state from /api/camera-power on mount and after
#   every toggle. The health-check loop also detects the "disabled" signal from
#   the backend (/health returns {disabled: true}) so external changes are caught.
#
#   Key invariant: the MJPEG <img> is NOT rendered when the camera is off.
#   This prevents ghost connections to a dead stream endpoint.
#
#   Status mapping (via GET to /health):
#     • 200 → "connected" (green)
#     • 503 + { disabled: true } → "off" (gray)
#     • 503 + { placeholder: true } → "no_camera" (blue)
#     • 503 otherwise → "connecting" (amber)
#     • network/CORS errors → "disconnected" (red)
*/

'use client';

import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Play, Pause, Maximize2, Minimize2, RotateCw, FlipHorizontal2,
  FlipVertical, SlidersHorizontal, Camera, Video, Square,
  Expand, Power, Loader2,
} from 'lucide-react';
import { cameraStatusBus } from '@/utils/cameraStatusBus';

type ServerStatus = 'connecting' | 'connected' | 'disconnected' | 'no_camera' | 'off';
type FitMode = 'cover' | 'contain';
type FilterMode = 'none' | 'mono' | 'contrast' | 'night';

interface CameraPowerState {
  enabled: boolean;
  healthy: boolean;
}

export interface CameraFrameProps {
  /** MJPEG stream URL, e.g. http(s)://robot/video_feed or /api/video-proxy?... */
  src: string;
  title?: string;
  className?: string;
  checkIntervalMs?: number;
  initialFit?: FitMode;
  showGrid?: boolean;
}

const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG;

// ------------------------------------------------------------------ //
//  Sub-components                                                      //
// ------------------------------------------------------------------ //

const StatusDot: React.FC<{ status: ServerStatus; title?: string }> = ({ status, title }) => {
  const color =
    status === 'connected'   ? 'bg-emerald-500' :
    status === 'connecting'  ? 'bg-amber-500'   :
    status === 'no_camera'   ? 'bg-sky-500'     :
    status === 'off'         ? 'bg-gray-500'    :
                               'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full shrink-0 ${color}`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
};

const IconBtn: React.FC<
  React.PropsWithChildren<{ onClick?: () => void; title?: string; disabled?: boolean; active?: boolean; danger?: boolean }>
> = ({ children, onClick, title, disabled, active, danger }) => (
  <button
    type="button"
    className={[
      'flex items-center justify-center w-7 h-7 rounded transition',
      'backdrop-blur border disabled:opacity-40 disabled:cursor-not-allowed',
      danger && active
        ? 'bg-rose-600/80 border-rose-400/60 text-white hover:bg-rose-500/90'
        : active
          ? 'bg-white/15 border-white/40 text-white'
          : 'bg-black/40 border-white/15 text-white/75 hover:bg-black/55 hover:text-white',
    ].join(' ')}
    onClick={onClick}
    title={title}
    aria-label={title}
    disabled={disabled}
  >
    {children}
  </button>
);

/** Prominent camera power pill button */
const PowerButton: React.FC<{
  enabled: boolean;
  loading: boolean;
  onClick: () => void;
}> = ({ enabled, loading, onClick }) => {
  return (
    <button
      type="button"
      onClick={onClick}
      disabled={loading}
      title={enabled ? 'Turn camera off' : 'Turn camera on'}
      aria-label={enabled ? 'Turn camera off' : 'Turn camera on'}
      className={[
        // base shape
        'relative flex items-center gap-1.5 px-3 h-8 rounded-full',
        'text-xs font-bold tracking-wide uppercase select-none',
        'border transition-all duration-200',
        'disabled:cursor-not-allowed disabled:opacity-60',
        // state-specific
        enabled
          ? [
              // ON → big red "turn off" button with glow
              'bg-rose-600 border-rose-400/80 text-white',
              'shadow-[0_0_14px_rgba(225,29,72,0.55)]',
              'hover:bg-rose-500 hover:shadow-[0_0_20px_rgba(225,29,72,0.75)]',
              'active:scale-95',
            ].join(' ')
          : [
              // OFF → muted green "turn on" button with subtle glow
              'bg-emerald-900/60 border-emerald-500/50 text-emerald-300',
              'shadow-[0_0_10px_rgba(16,185,129,0.25)]',
              'hover:bg-emerald-800/70 hover:border-emerald-400/70 hover:shadow-[0_0_16px_rgba(16,185,129,0.45)]',
              'active:scale-95',
            ].join(' '),
      ].join(' ')}
    >
      {loading
        ? <Loader2 size={12} className="animate-spin shrink-0" />
        : <Power size={12} className="shrink-0" />}
      <span>{enabled ? 'Cam Off' : 'Cam On'}</span>

      {/* Pulsing dot when camera is running */}
      {enabled && !loading && (
        <span className="absolute -top-0.5 -right-0.5 w-2 h-2 rounded-full bg-rose-400 animate-pulse" />
      )}
    </button>
  );
};

// ------------------------------------------------------------------ //
//  Helpers                                                             //
// ------------------------------------------------------------------ //

const filterClass = (mode: FilterMode) => {
  switch (mode) {
    case 'mono':     return 'filter grayscale';
    case 'contrast': return 'filter contrast-200 brightness-90';
    case 'night':    return 'filter contrast-150 saturate-150 hue-rotate-60';
    default:         return '';
  }
};

const buildHealthUrl = (srcUrl?: string) => {
  if (!srcUrl) return '';
  try {
    const base = typeof window !== 'undefined' ? window.location.origin : 'http://localhost';
    const u = new URL(srcUrl, base);
    if (u.pathname.startsWith('/api/video-proxy')) {
      const qs = new URLSearchParams(u.search);
      qs.delete('b');
      const q = qs.toString();
      return `/api/video-health${q ? `?${q}` : ''}`;
    }
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = '';
      return u.toString();
    }
    u.pathname = (u.pathname.replace(/\/+$/, '') || '') + '/health';
    return u.toString();
  } catch { return ''; }
};

async function fetchWithTimeout(url: string, ms: number) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), Math.max(500, ms));
  try {
    return await fetch(url, { method: 'GET', cache: 'no-store', signal: ac.signal });
  } finally {
    clearTimeout(t);
  }
}

// ------------------------------------------------------------------ //
//  Component                                                           //
// ------------------------------------------------------------------ //

const CameraFrame: React.FC<CameraFrameProps> = ({
  src,
  title = 'Camera',
  className = '',
  checkIntervalMs = 5000,
  initialFit = 'cover',
  showGrid = false,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const imgRef       = useRef<HTMLImageElement | null>(null);
  const canvasRef    = useRef<HTMLCanvasElement | null>(null);

  // Server status (derived from /health polling)
  const [status,   setStatus]   = useState<ServerStatus>('connecting');
  const [pingMs,   setPingMs]   = useState<number | null>(null);
  const [lastHttp, setLastHttp] = useState<number | null>(null);
  const [fps,      setFps]      = useState<number>(0);

  // Authoritative camera power state (from /api/camera-power)
  const [camState,     setCamState]     = useState<CameraPowerState>({ enabled: true, healthy: false });
  const [powerLoading, setPowerLoading] = useState(false);

  // Stream / UI state
  const [playing,  setPlaying]  = useState(true);
  const [fitMode,  setFitMode]  = useState<FitMode>(initialFit);
  const [rotate,   setRotate]   = useState<0 | 90 | 180 | 270>(0);
  const [flipH,    setFlipH]    = useState(false);
  const [flipV,    setFlipV]    = useState(false);
  const [filter,   setFilter]   = useState<FilterMode>('none');

  const [snapshotUrl,   setSnapshotUrl]   = useState<string | null>(null);
  const lastBlobUrlRef = useRef<string | null>(null);

  const recStreamRef  = useRef<MediaStream | null>(null);
  const recorderRef   = useRef<MediaRecorder | null>(null);
  const chunksRef     = useRef<Blob[]>([]);
  const [recording, setRecording] = useState(false);

  const rafRef        = useRef<number | null>(null);
  const fpsCounterRef = useRef<{ last: number; frames: number }>({
    last: performance.now?.() ?? Date.now(), frames: 0,
  });
  const retryTimerRef = useRef<number | null>(null);
  const backoffRef    = useRef<number>(800);

  const [buster, setBuster] = useState<number>(() => Date.now());
  const IMG_SRC = useMemo(() => {
    if (!src) return '';
    const sep = src.includes('?') ? '&' : '?';
    return `${src}${sep}b=${buster}`;
  }, [src, buster]);

  // ---------------------------------------------------------------- //
  //  Camera power — fetch authoritative state from backend            //
  // ---------------------------------------------------------------- //

  const fetchCameraState = useCallback(async () => {
    try {
      const res = await fetch('/api/camera-power', { cache: 'no-store' });
      const data = await res.json().catch(() => null);
      if (data && typeof data.enabled === 'boolean') {
        setCamState({ enabled: data.enabled, healthy: !!data.healthy });
        if (!data.enabled) setStatus('off');
      }
    } catch (e) {
      if (DEBUG) console.warn('[CameraFrame] camera-power fetch error:', e);
    }
  }, []);

  // Fetch authoritative state once on mount
  useEffect(() => { fetchCameraState(); }, [fetchCameraState]);

  // ---------------------------------------------------------------- //
  //  Status bus publish helper                                        //
  // ---------------------------------------------------------------- //

  const publish = (s: Partial<Parameters<typeof cameraStatusBus.publish>[0]>) => {
    cameraStatusBus.publish({
      state:   (s.state as any) ?? status,
      pingMs:  s.pingMs  ?? pingMs,
      lastHttp: s.lastHttp ?? lastHttp,
      playing: s.playing ?? playing,
      ts:      Date.now(),
    });
  };

  // ---------------------------------------------------------------- //
  //  Health check polling                                             //
  // ---------------------------------------------------------------- //

  useEffect(() => {
    let cancelled = false;
    const healthUrl = buildHealthUrl(src);

    const check = async () => {
      if (!healthUrl) {
        if (!cancelled) {
          setStatus('disconnected'); setPingMs(null); setLastHttp(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: null });
        }
        return;
      }
      try {
        setStatus(s => (s === 'connected' || s === 'no_camera') ? s : 'connecting');

        const start = performance.now?.() ?? Date.now();
        const res   = await fetchWithTimeout(healthUrl, Math.max(1500, checkIntervalMs - 500));
        const end   = performance.now?.() ?? Date.now();
        if (cancelled) return;

        setLastHttp(res.status);
        let body: any = null;
        try { body = await res.clone().json(); } catch {}

        const rtt = Math.max(0, Math.round(end - start));

        if (res.ok) {
          setCamState(s => ({ ...s, enabled: true }));
          setStatus('connected'); setPingMs(rtt);
          publish({ state: 'connected', pingMs: rtt, lastHttp: res.status });
        } else if (res.status === 503 && body?.disabled === true) {
          // Backend explicitly disabled — update both state and status
          setCamState({ enabled: false, healthy: false });
          setStatus('off'); setPingMs(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: res.status });
        } else if (res.status === 503 && body?.placeholder === true) {
          setCamState(s => ({ ...s, enabled: true }));
          setStatus('no_camera'); setPingMs(rtt);
          publish({ state: 'no_camera', pingMs: rtt, lastHttp: res.status });
        } else if (res.status === 503) {
          setCamState(s => ({ ...s, enabled: true }));
          setStatus('connecting'); setPingMs(rtt);
          publish({ state: 'connecting', pingMs: rtt, lastHttp: res.status });
        } else {
          setCamState(s => ({ ...s, enabled: true }));
          setStatus('disconnected'); setPingMs(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: res.status });
        }
      } catch (e) {
        if (!cancelled) {
          if (DEBUG) console.warn('[CameraFrame] /health fetch error:', e);
          setStatus('disconnected'); setPingMs(null); setLastHttp(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: null });
        }
      }
    };

    check();
    const t = window.setInterval(check, Math.max(2000, checkIntervalMs));
    return () => { cancelled = true; window.clearInterval(t); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [src, checkIntervalMs]);

  // ---------------------------------------------------------------- //
  //  FPS counter (rAF loop — only when recording or snapshotting)    //
  // ---------------------------------------------------------------- //

  useEffect(() => {
    if (!(recording || snapshotUrl)) {
      setFps(0);
      if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current = null; }
      return;
    }
    const loop = () => {
      const img = imgRef.current, cvs = canvasRef.current;
      if (img && cvs && (recording || snapshotUrl)) {
        const w = img.naturalWidth || img.clientWidth || 640;
        const h = img.naturalHeight || img.clientHeight || 360;
        if (cvs.width !== w || cvs.height !== h) { cvs.width = w; cvs.height = h; }
        const ctx = cvs.getContext('2d');
        if (ctx) {
          ctx.save();
          ctx.clearRect(0, 0, cvs.width, cvs.height);
          try { ctx.drawImage(img, 0, 0, cvs.width, cvs.height); } catch {}
          ctx.fillStyle = 'rgba(0,0,0,0.5)'; ctx.fillRect(8, 8, 230, 22);
          ctx.fillStyle = '#fff'; ctx.font = '14px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas';
          ctx.fillText(new Date().toLocaleString(), 14, 24);
          if (showGrid) {
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.beginPath();
            ctx.moveTo(cvs.width / 2, 0); ctx.lineTo(cvs.width / 2, cvs.height);
            ctx.moveTo(0, cvs.height / 2); ctx.lineTo(cvs.width, cvs.height / 2);
            ctx.stroke();
          }
          ctx.restore();
        }
        const now = performance.now?.() ?? Date.now();
        fpsCounterRef.current.frames += 1;
        if (now - fpsCounterRef.current.last >= 1000) {
          setFps(fpsCounterRef.current.frames);
          fpsCounterRef.current.frames = 0;
          fpsCounterRef.current.last   = now;
        }
      }
      rafRef.current = requestAnimationFrame(loop);
    };
    rafRef.current = requestAnimationFrame(loop);
    return () => { if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current = null; } };
  }, [recording, snapshotUrl, showGrid]);

  // ---------------------------------------------------------------- //
  //  Helpers                                                          //
  // ---------------------------------------------------------------- //

  const clearRetryTimer = () => {
    if (retryTimerRef.current) { window.clearTimeout(retryTimerRef.current); retryTimerRef.current = null; }
  };
  const resetBackoff = () => { backoffRef.current = 800; clearRetryTimer(); };
  const scheduleRetry = () => {
    clearRetryTimer();
    const delay = Math.min(6000, backoffRef.current);
    retryTimerRef.current = window.setTimeout(() => {
      setBuster(Date.now());
      if (DEBUG) console.log('[CameraFrame] retrying MJPEG after', delay, 'ms');
      backoffRef.current = Math.min(6000, Math.floor(backoffRef.current * 1.6));
    }, delay);
  };

  const ensureCanvasDraw = async () => {
    const img = imgRef.current, cvs = canvasRef.current;
    if (!img || !cvs) return;
    try {
      const w = img.naturalWidth || img.clientWidth || 640;
      const h = img.naturalHeight || img.clientHeight || 360;
      cvs.width = w; cvs.height = h;
      const ctx = cvs.getContext('2d'); if (!ctx) return;
      ctx.drawImage(img, 0, 0, w, h);
      ctx.fillStyle = 'rgba(0,0,0,0.5)'; ctx.fillRect(8, 8, 230, 22);
      ctx.fillStyle = '#fff'; ctx.font = '14px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas';
      ctx.fillText(new Date().toLocaleString(), 14, 24);
    } catch { /* CORS-blocked draws will throw; ignore. */ }
  };

  // ---------------------------------------------------------------- //
  //  Play / Pause                                                     //
  // ---------------------------------------------------------------- //

  const handlePause = async () => {
    await ensureCanvasDraw();
    try {
      const cvs = canvasRef.current;
      if (cvs) {
        cvs.toBlob((blob) => {
          if (lastBlobUrlRef.current) { URL.revokeObjectURL(lastBlobUrlRef.current); lastBlobUrlRef.current = null; }
          if (blob) {
            const url = URL.createObjectURL(blob);
            lastBlobUrlRef.current = url;
            setSnapshotUrl(url);
          } else {
            setSnapshotUrl(cvs.toDataURL('image/png'));
          }
        }, 'image/png');
      }
    } catch { setSnapshotUrl(canvasRef.current?.toDataURL('image/png') || null); }
    try { if (imgRef.current) imgRef.current.src = ''; } catch {}
    setPlaying(false);
    publish({ playing: false });
    clearRetryTimer();
  };

  const handlePlay = () => {
    if (lastBlobUrlRef.current) { URL.revokeObjectURL(lastBlobUrlRef.current); lastBlobUrlRef.current = null; }
    setSnapshotUrl(null);
    setPlaying(true);
    publish({ playing: true });
    resetBackoff();
    setBuster(Date.now());
  };

  const handleTogglePlay = () => (playing ? handlePause() : handlePlay());

  // ---------------------------------------------------------------- //
  //  Camera power toggle                                              //
  // ---------------------------------------------------------------- //

  const handleTogglePower = async () => {
    if (powerLoading) return;
    const next = !camState.enabled;
    setPowerLoading(true);
    // Optimistic update
    setCamState(s => ({ ...s, enabled: next }));
    if (!next) { setStatus('off'); handlePause(); }
    try {
      await fetch('/api/camera-power', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: next }),
        cache: 'no-store',
      });
      // Re-fetch authoritative state after backend confirms
      await fetchCameraState();
      if (next) handlePlay();
    } catch (e) {
      if (DEBUG) console.warn('[CameraFrame] power toggle error:', e);
      // Roll back optimistic update on failure
      setCamState(s => ({ ...s, enabled: !next }));
      if (!next) setStatus('connecting');
    } finally {
      setPowerLoading(false);
    }
  };

  // ---------------------------------------------------------------- //
  //  Snapshot / Record                                                //
  // ---------------------------------------------------------------- //

  const handleSnapshotDownload = async () => {
    if (!(status === 'connected' && playing)) return;
    await ensureCanvasDraw();
    const cvs = canvasRef.current; if (!cvs) return;
    try {
      const a = document.createElement('a');
      a.href = cvs.toDataURL('image/png');
      a.download = `snapshot-${new Date().toISOString().replace(/[:.]/g, '-')}.png`;
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
    } catch (e) { console.warn('Snapshot blocked (CORS). Same-origin or CORS required.', e); }
  };

  const startRecording = () => {
    if (!(status === 'connected' && playing)) return;
    const cvs = canvasRef.current; if (!cvs) return;
    try {
      const stream = cvs.captureStream(30);
      const mimeOptions = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm'];
      let mimeType = '';
      for (const m of mimeOptions) {
        if ((window as any).MediaRecorder?.isTypeSupported?.(m)) { mimeType = m; break; }
      }
      const rec = new MediaRecorder(stream, mimeType ? { mimeType } : undefined);
      chunksRef.current = [];
      rec.ondataavailable = (e) => { if (e.data?.size > 0) chunksRef.current.push(e.data); };
      rec.onstop = () => {
        try {
          const blob = new Blob(chunksRef.current, { type: mimeType || 'video/webm' });
          const url  = URL.createObjectURL(blob);
          const a    = document.createElement('a');
          a.href = url; a.download = `recording-${new Date().toISOString().replace(/[:.]/g, '-')}.webm`;
          document.body.appendChild(a); a.click(); document.body.removeChild(a);
          setTimeout(() => URL.revokeObjectURL(url), 10000);
        } catch (e) { console.warn('Recording finalize failed:', e); }
      };
      rec.start();
      recStreamRef.current = stream;
      recorderRef.current  = rec;
      setRecording(true);
    } catch (err) { console.warn('Recording failed:', err); setRecording(false); }
  };

  const stopRecording = () => {
    try { recorderRef.current?.stop(); } catch {}
    try { recStreamRef.current?.getTracks().forEach(t => t.stop()); } catch {}
    recorderRef.current = null; recStreamRef.current = null; setRecording(false);
  };

  // ---------------------------------------------------------------- //
  //  View transform                                                   //
  // ---------------------------------------------------------------- //

  const mediaTransform = useMemo(() => {
    const sx = flipH ? -1 : 1, sy = flipV ? -1 : 1;
    return `rotate(${rotate}deg) scale(${sx}, ${sy})`;
  }, [rotate, flipH, flipV]);

  const mediaStyle: React.CSSProperties = useMemo(
    () => ({ transform: mediaTransform, transformOrigin: 'center center' }),
    [mediaTransform],
  );

  // ---------------------------------------------------------------- //
  //  Image load / error                                               //
  // ---------------------------------------------------------------- //

  const handleImgLoad = () => {
    setStatus('connected');
    publish({ state: 'connected' });
    resetBackoff();
    if (DEBUG) console.log('[CameraFrame] <img> load OK');
  };

  const handleImgError = (e?: any) => {
    setStatus(prev => prev === 'no_camera' ? prev : 'disconnected');
    publish({ state: 'disconnected' });
    if (DEBUG) console.warn('[CameraFrame] <img> error', e?.message || e);
    scheduleRetry();
  };

  // Cleanup
  useEffect(() => () => {
    clearRetryTimer();
    try { recorderRef.current?.stop(); } catch {}
    try { recStreamRef.current?.getTracks().forEach(t => t.stop()); } catch {}
    recorderRef.current = null; recStreamRef.current = null;
    if (lastBlobUrlRef.current) { URL.revokeObjectURL(lastBlobUrlRef.current); lastBlobUrlRef.current = null; }
    if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current = null; }
  }, []);

  // ---------------------------------------------------------------- //
  //  Derived display values                                           //
  // ---------------------------------------------------------------- //

  const titleText =
    status === 'connected'  ? `Video: Connected • ${pingMs ?? '—'} ms` :
    status === 'no_camera'  ? 'Video: No camera'                        :
    status === 'connecting' ? 'Video: Connecting…'                      :
    status === 'off'        ? 'Video: Off'                              :
                              'Video: Disconnected';

  // The MJPEG <img> must not render when camera is off — prevents ghost streams.
  const showStream = camState.enabled && playing;

  // ---------------------------------------------------------------- //
  //  Render                                                           //
  // ---------------------------------------------------------------- //

  return (
    <div
      ref={containerRef}
      className={`relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden ${className}`}
    >
      {/* ── Header ──────────────────────────────────────────────────── */}
      <div className="flex items-center justify-between px-3 py-2 bg-black/40 backdrop-blur border-b border-white/10">

        {/* Left: status dot + title + stats */}
        <div className="flex items-center gap-2 min-w-0">
          <StatusDot status={status} title={titleText} />
          <span className="text-sm font-semibold text-white truncate">{title}</span>
          <span className="text-xs text-white/50 tabular-nums shrink-0">
            {pingMs != null ? `${pingMs}ms` : '—'}{fps > 0 ? ` • ${fps}fps` : ''}
          </span>
        </div>

        {/* Right: icon controls + prominent power button */}
        <div className="flex items-center gap-1 shrink-0">

          {/* Playback */}
          <IconBtn onClick={handleTogglePlay} title={playing ? 'Pause stream' : 'Resume stream'} disabled={!camState.enabled}>
            {playing ? <Pause size={13} /> : <Play size={13} />}
          </IconBtn>

          <span className="w-px h-4 bg-white/10 mx-0.5" />

          {/* View transforms */}
          <IconBtn
            onClick={() => setFitMode(fitMode === 'cover' ? 'contain' : 'cover')}
            title={fitMode === 'cover' ? 'Switch to contain' : 'Switch to cover'}
            active={fitMode === 'contain'}
          >
            {fitMode === 'cover' ? <Maximize2 size={13} /> : <Minimize2 size={13} />}
          </IconBtn>
          <IconBtn onClick={() => setRotate(r => ((r + 90) % 360) as 0 | 90 | 180 | 270)} title="Rotate 90°">
            <RotateCw size={13} />
          </IconBtn>
          <IconBtn onClick={() => setFlipH(v => !v)} title="Flip horizontal" active={flipH}>
            <FlipHorizontal2 size={13} />
          </IconBtn>
          <IconBtn onClick={() => setFlipV(v => !v)} title="Flip vertical" active={flipV}>
            <FlipVertical size={13} />
          </IconBtn>
          <IconBtn
            onClick={() => setFilter(f =>
              f === 'none' ? 'mono' : f === 'mono' ? 'contrast' : f === 'contrast' ? 'night' : 'none'
            )}
            title={`Filter: ${filter} (click to cycle)`}
            active={filter !== 'none'}
          >
            <SlidersHorizontal size={13} />
          </IconBtn>

          <span className="w-px h-4 bg-white/10 mx-0.5" />

          {/* Capture */}
          <IconBtn
            onClick={handleSnapshotDownload}
            title="Save snapshot (PNG)"
            disabled={!(status === 'connected' && playing)}
          >
            <Camera size={13} />
          </IconBtn>

          {/* Record clip */}
          <div className="relative flex items-center">
            <IconBtn
              onClick={recording ? stopRecording : startRecording}
              title={recording ? 'Stop recording' : 'Record clip (.webm)'}
              active={recording}
              danger={recording}
              disabled={!recording && !(status === 'connected' && playing)}
            >
              {recording ? <Square size={13} /> : <Video size={13} />}
            </IconBtn>
            {recording && (
              <span className="absolute -top-1.5 -right-1.5 w-2 h-2 rounded-full bg-rose-500 animate-pulse shadow-[0_0_5px_rgba(244,63,94,0.8)]" />
            )}
          </div>
          <span className="text-[9px] text-white/30 font-medium -ml-0.5 select-none hidden sm:inline">
            {recording ? 'REC' : 'Clip'}
          </span>

          <span className="w-px h-4 bg-white/10 mx-0.5" />

          {/* Fullscreen */}
          <IconBtn
            onClick={() => {
              const el = containerRef.current; if (!el) return;
              if (document.fullscreenElement) document.exitFullscreen().catch(() => {});
              else el.requestFullscreen?.().catch(() => {});
            }}
            title="Toggle fullscreen"
          >
            <Expand size={13} />
          </IconBtn>

          {/* ── Prominent power button (larger + labelled) ─────────── */}
          <span className="w-px h-4 bg-white/10 mx-1" />
          <PowerButton
            enabled={camState.enabled}
            loading={powerLoading}
            onClick={handleTogglePower}
          />
        </div>
      </div>

      {/* ── Media area (16:9) ──────────────────────────────────────── */}
      <div className={`relative bg-black ${filterClass(filter)}`} style={{ paddingTop: '56.25%' }}>

        {/* Live stream — only rendered when camera is enabled */}
        {showStream && (
          // eslint-disable-next-line @next/next/no-img-element
          <img
            ref={imgRef}
            key={IMG_SRC}
            src={IMG_SRC}
            alt="Live Camera"
            className={`absolute inset-0 w-full h-full ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
            style={mediaStyle}
            onError={handleImgError}
            onLoad={handleImgLoad}
            draggable={false}
          />
        )}

        {/* Paused — frozen snapshot */}
        {!playing && camState.enabled && snapshotUrl && (
          // eslint-disable-next-line @next/next/no-img-element
          <img
            src={snapshotUrl}
            alt="Last frame"
            className={`absolute inset-0 w-full h-full opacity-25 ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
            style={mediaStyle}
            draggable={false}
          />
        )}

        {/* Paused with no snapshot */}
        {!playing && camState.enabled && !snapshotUrl && (
          <div className="absolute inset-0 flex items-center justify-center text-white/70 text-sm">
            Paused
          </div>
        )}

        {/* Disconnected retry */}
        {status === 'disconnected' && playing && camState.enabled && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-3">Not connected</div>
            <button
              type="button"
              className="px-2 py-1 rounded bg-black/40 hover:bg-black/55 text-white text-xs backdrop-blur border border-white/15 transition"
              onClick={handlePlay}
            >
              Retry
            </button>
          </div>
        )}

        {/* No camera hardware */}
        {status === 'no_camera' && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-1">Camera missing</div>
            <div className="text-xs text-white/70">Check ribbon cable / device</div>
          </div>
        )}

        {/* Camera off overlay */}
        {status === 'off' && (
          <div className="absolute inset-0 flex flex-col items-center justify-center gap-4 bg-black/70">
            <div className="flex flex-col items-center gap-2">
              <div className="w-14 h-14 rounded-full bg-gray-800 border border-gray-600 flex items-center justify-center">
                <Power size={24} className="text-gray-400" />
              </div>
              <span className="text-white/60 text-sm font-medium tracking-wide">Camera is off</span>
            </div>
            <PowerButton enabled={false} loading={powerLoading} onClick={handleTogglePower} />
          </div>
        )}
      </div>

      {/* Hidden canvas for snapshot / recording */}
      <canvas ref={canvasRef} className="hidden" />
    </div>
  );
};

export default CameraFrame;
