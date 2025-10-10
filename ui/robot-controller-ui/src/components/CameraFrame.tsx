/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/CameraFrame.tsx
# Summary:
#   MJPEG camera frame with controls (play/pause, fit, rotate, flip, filters,
#   snapshot/record, fullscreen) and a status dot that matches the status bar.
#
#   Improvements in this pass:
#   - Use computed transform (`mediaTransform`) consistently for <img>.
#   - rAF loop only runs when recording or a snapshot is shown (CPU saver).
#   - Extra guardrails around /health fetch (AbortController timeout).
#   - More robust cleanup of timers, object-URLs and recorder streams.
#   - Minor a11y/title polish and clearer debug logs.
#
#   Status mapping (via GET to /health):
#     • 200 → "connected" (green)
#     • 503 + { placeholder: true } → "no_camera" (blue)
#     • 503 otherwise → "connecting" (amber)
#     • network/CORS errors → "disconnected" (red)
*/

'use client';

import React, { useEffect, useMemo, useRef, useState } from 'react';
import Image from 'next/image';
import { cameraStatusBus } from '@/utils/cameraStatusBus';

type ServerStatus = 'connecting' | 'connected' | 'disconnected' | 'no_camera';
type FitMode = 'cover' | 'contain';
type FilterMode = 'none' | 'mono' | 'contrast' | 'night';

export interface CameraFrameProps {
  /** MJPEG stream URL, e.g. http(s)://robot/video_feed or /api/video-proxy?... */
  src: string;
  title?: string;
  className?: string;
  /** latency poll interval */
  checkIntervalMs?: number;
  /** initial object-fit for the video */
  initialFit?: FitMode;
  /** show a faint crosshair grid overlay while recording/snapshotting */
  showGrid?: boolean;
}

const DEBUG = !!process.env.NEXT_PUBLIC_WS_DEBUG;

const StatusDot: React.FC<{ status: ServerStatus; title?: string }> = ({ status, title }) => {
  const color =
    status === 'connected'   ? 'bg-emerald-500' :
    status === 'connecting'  ? 'bg-amber-500'  :
    status === 'no_camera'   ? 'bg-sky-500'    :
                               'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }}
      title={title}
      aria-label={title}
    />
  );
};

const IconBtn: React.FC<
  React.PropsWithChildren<{ onClick?: () => void; title?: string; disabled?: boolean; active?: boolean }>
> = ({ children, onClick, title, disabled, active }) => (
  <button
    type="button"
    className={`px-2 py-1 rounded bg-black/40 hover:bg-black/55 text-white text-xs backdrop-blur
                border ${active ? 'border-white/50' : 'border-white/15'} transition
                disabled:opacity-40 disabled:cursor-not-allowed`}
    onClick={onClick}
    title={title}
    aria-label={title}
    disabled={disabled}
  >
    {children}
  </button>
);

/** Tailwind filter presets */
const filterClass = (mode: FilterMode) => {
  switch (mode) {
    case 'mono': return 'filter grayscale';
    case 'contrast': return 'filter contrast-200 brightness-90';
    case 'night': return 'filter contrast-150 saturate-150 hue-rotate-60';
    default: return '';
  }
};

/** Build the correct health URL for whatever `src` we were given. */
const buildHealthUrl = (srcUrl?: string) => {
  if (!srcUrl) return '';
  try {
    const base = typeof window !== 'undefined' ? window.location.origin : 'http://localhost';
    const u = new URL(srcUrl, base);

    // If same-origin proxy, mirror its query to /api/video-health (and drop our cache-buster)
    if (u.pathname.startsWith('/api/video-proxy')) {
      const qs = new URLSearchParams(u.search);
      qs.delete('b');
      const q = qs.toString();
      return `/api/video-health${q ? `?${q}` : ''}`;
    }

    // Direct upstream:
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = '';
      return u.toString();
    }
    u.pathname = (u.pathname.replace(/\/+$/,'') || '') + '/health';
    return u.toString();
  } catch {
    return '';
  }
};

/** fetch with timeout guard (component-local) */
async function fetchWithTimeout(url: string, ms: number) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), Math.max(500, ms));
  try {
    const res = await fetch(url, { method: 'GET', cache: 'no-store', signal: ac.signal });
    return res;
  } finally {
    clearTimeout(t);
  }
}

const CameraFrame: React.FC<CameraFrameProps> = ({
  src,
  title = 'Camera',
  className = '',
  checkIntervalMs = 5000,
  initialFit = 'cover',
  showGrid = false,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  const [status, setStatus] = useState<ServerStatus>('connecting');
  const [pingMs, setPingMs] = useState<number | null>(null);
  const [lastHttp, setLastHttp] = useState<number | null>(null);
  const [fps, setFps] = useState<number>(0);

  const [playing, setPlaying] = useState(true);
  const [fitMode, setFitMode] = useState<FitMode>(initialFit);
  const [rotate, setRotate] = useState<0 | 90 | 180 | 270>(0);
  const [flipH, setFlipH] = useState(false);
  const [flipV, setFlipV] = useState(false);
  const [filter, setFilter] = useState<FilterMode>('none');

  const [snapshotUrl, setSnapshotUrl] = useState<string | null>(null);
  const lastBlobUrlRef = useRef<string | null>(null);

  const recStreamRef = useRef<MediaStream | null>(null);
  const recorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);
  const [recording, setRecording] = useState(false);

  const rafRef = useRef<number | null>(null);
  const fpsCounterRef = useRef<{ last: number; frames: number }>({ last: performance.now?.() ?? Date.now(), frames: 0 });

  const retryTimerRef = useRef<number | null>(null);
  const backoffRef = useRef<number>(800); // ms; grows up to ~6s

  // ---- Stable cache-buster: only changes on play/retry/backoff ----
  const [buster, setBuster] = useState<number>(() => Date.now());
  const IMG_SRC = useMemo(() => {
    if (!src) return '';
    const sep = src.includes('?') ? '&' : '?';
    return `${src}${sep}b=${buster}`;
  }, [src, buster]);

  // Push current status to the bus
  const publish = (s: Partial<Parameters<typeof cameraStatusBus.publish>[0]>) => {
    cameraStatusBus.publish({
      state: (s.state as any) ?? status,
      pingMs: s.pingMs ?? pingMs,
      lastHttp: s.lastHttp ?? lastHttp,
      playing: s.playing ?? playing,
      ts: Date.now(),
    });
  };

  // ---------- Availability + latency via GET /health ----------
  useEffect(() => {
    let cancelled = false;
    const healthUrl = buildHealthUrl(src);

    const check = async () => {
      if (!healthUrl) {
        if (!cancelled) {
          setStatus('disconnected');
          setPingMs(null);
          setLastHttp(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: null });
        }
        return;
      }
      try {
        setStatus(s => (s === 'connected' || s === 'no_camera') ? s : 'connecting');
        publish({ state: 'connecting' });

        const start = performance.now?.() ?? Date.now();
        const res = await fetchWithTimeout(healthUrl, Math.max(1500, checkIntervalMs - 500));
        const end = performance.now?.() ?? Date.now();
        if (cancelled) return;

        setLastHttp(res.status);

        let body: any = null;
        try { body = await res.clone().json(); } catch {}

        const rtt = Math.max(0, Math.round(end - start));
        if (res.ok) {
          setStatus('connected'); setPingMs(rtt);
          publish({ state: 'connected', pingMs: rtt, lastHttp: res.status });
        } else if (res.status === 503 && body && body.placeholder === true) {
          setStatus('no_camera'); setPingMs(rtt);
          publish({ state: 'no_camera', pingMs: rtt, lastHttp: res.status });
        } else if (res.status === 503) {
          setStatus('connecting'); setPingMs(rtt);
          publish({ state: 'connecting', pingMs: rtt, lastHttp: res.status });
        } else {
          setStatus('disconnected'); setPingMs(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: res.status });
        }
      } catch (e) {
        if (!cancelled) {
          if (DEBUG) console.warn('[CameraFrame] /health fetch error:', e);
          setStatus('disconnected'); setPingMs(null);
          setLastHttp(null);
          publish({ state: 'disconnected', pingMs: null, lastHttp: null });
        }
      }
    };

    check();
    const t = window.setInterval(check, Math.max(2000, checkIntervalMs));
    return () => { cancelled = true; window.clearInterval(t); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [src, checkIntervalMs]);

  // ---------- FPS estimate (only while recording or snapshot to canvas) ----------
  useEffect(() => {
    if (!(recording || snapshotUrl)) {
      // reset fps to 0 when not drawing
      setFps(0);
      if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current = null; }
      return;
    }

    const loop = () => {
      const img = imgRef.current;
      const cvs = canvasRef.current;
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
          fpsCounterRef.current.last = now;
        }
      }
      rafRef.current = requestAnimationFrame(loop);
    };
    rafRef.current = requestAnimationFrame(loop);
    return () => { if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current = null; } };
  }, [recording, snapshotUrl, showGrid]);

  // ---------- helpers ----------
  const clearRetryTimer = () => {
    if (retryTimerRef.current) {
      window.clearTimeout(retryTimerRef.current);
      retryTimerRef.current = null;
    }
  };

  const resetBackoff = () => { backoffRef.current = 800; clearRetryTimer(); };

  const scheduleRetry = () => {
    clearRetryTimer();
    const delay = Math.min(6000, backoffRef.current);
    retryTimerRef.current = window.setTimeout(() => {
      setBuster(Date.now());
      // Only log retry attempts in debug mode to reduce console spam
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
    } catch {
      // CORS-blocked draws will throw; ignore.
    }
  };

  // ---------- Play/Pause ----------
  const handlePause = async () => {
    await ensureCanvasDraw();

    // Produce snapshot (prefer Blob URL; fallback to data URL)
    try {
      const cvs = canvasRef.current;
      if (cvs) {
        cvs.toBlob((blob) => {
          if (lastBlobUrlRef.current) {
            URL.revokeObjectURL(lastBlobUrlRef.current);
            lastBlobUrlRef.current = null;
          }
          if (blob) {
            const url = URL.createObjectURL(blob);
            lastBlobUrlRef.current = url;
            setSnapshotUrl(url);
          } else {
            setSnapshotUrl(cvs.toDataURL('image/png'));
          }
        }, 'image/png');
      }
    } catch {
      setSnapshotUrl(canvasRef.current?.toDataURL('image/png') || null);
    }

    try { if (imgRef.current) imgRef.current.src = ''; } catch {}
    setPlaying(false);
    publish({ playing: false });
    clearRetryTimer();
  };

  const handlePlay = () => {
    // drop previous snapshot and revoke any blob URL
    if (lastBlobUrlRef.current) {
      URL.revokeObjectURL(lastBlobUrlRef.current);
      lastBlobUrlRef.current = null;
    }
    setSnapshotUrl(null);
    setPlaying(true);
    publish({ playing: true });
    resetBackoff();
    setBuster(Date.now()); // force new request once
  };

  const handleTogglePlay = () => (playing ? handlePause() : handlePlay());

  // ---------- Snapshot PNG (download) ----------
  const handleSnapshotDownload = async () => {
    if (!(status === 'connected' && playing)) return;
    await ensureCanvasDraw();
    const cvs = canvasRef.current; if (!cvs) return;
    try {
      const dataUrl = cvs.toDataURL('image/png');
      const a = document.createElement('a');
      a.href = dataUrl;
      a.download = `snapshot-${new Date().toISOString().replace(/[:.]/g, '-')}.png`;
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
    } catch (e) {
      console.warn('Snapshot blocked (likely CORS). Same-origin or proper CORS is required.', e);
    }
  };

  // ---------- Record WebM via canvas.captureStream ----------
  const startRecording = () => {
    if (!(status === 'connected' && playing)) return;
    const cvs = canvasRef.current; if (!cvs) return;
    try {
      const stream = cvs.captureStream(30);
      const mimeOptions = ['video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm'];
      let mimeType = '';
      for (const m of mimeOptions) {
        if ((window as any).MediaRecorder?.isTypeSupported?.(m)) { mimeType = m; break; }
      }
      const rec = new MediaRecorder(stream, mimeType ? { mimeType } : undefined);
      chunksRef.current = [];
      rec.ondataavailable = (e) => { if (e.data && e.data.size > 0) chunksRef.current.push(e.data); };
      rec.onstop = () => {
        try {
          const blob = new Blob(chunksRef.current, { type: mimeType || 'video/webm' });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url; a.download = `recording-${new Date().toISOString().replace(/[:.]/g, '-')}.webm`;
          document.body.appendChild(a); a.click(); document.body.removeChild(a);
          setTimeout(() => URL.revokeObjectURL(url), 10000);
        } catch (e) {
          console.warn('Recording finalize failed:', e);
        }
      };
      rec.start();
      recStreamRef.current = stream;
      recorderRef.current = rec;
      setRecording(true);
    } catch (err) {
      console.warn('Recording failed (likely CORS or browser support):', err);
      setRecording(false);
    }
  };

  const stopRecording = () => {
    try { recorderRef.current?.stop(); } catch {}
    try { recStreamRef.current?.getTracks().forEach(t => t.stop()); } catch {}
    recorderRef.current = null; recStreamRef.current = null; setRecording(false);
  };

  // ---------- View transform ----------
  const mediaTransform = useMemo(() => {
    const r = rotate;
    const sx = flipH ? -1 : 1;
    const sy = flipV ? -1 : 1;
    return `rotate(${r}deg) scale(${sx}, ${sy})`;
  }, [rotate, flipH, flipV]);

  const mediaStyle: React.CSSProperties = useMemo(
    () => ({ transform: mediaTransform, transformOrigin: 'center center' }),
    [mediaTransform]
  );

  // ---------- Image load/error → status + retries ----------
  const handleImgLoad = () => {
    setStatus('connected');
    publish({ state: 'connected' });
    resetBackoff();
    if (DEBUG) console.log('[CameraFrame] <img> load OK');
  };

  const handleImgError = (e?: any) => {
    setStatus(prev => (prev === 'no_camera' ? prev : 'disconnected'));
    publish({ state: 'disconnected' });
    if (DEBUG) console.warn('[CameraFrame] <img> error', e?.message || e);
    scheduleRetry();
  };

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      clearRetryTimer();
      try { recorderRef.current?.stop(); } catch {}
      try { recStreamRef.current?.getTracks().forEach(t => t.stop()); } catch {}
      recorderRef.current = null; recStreamRef.current = null;
      if (lastBlobUrlRef.current) { URL.revokeObjectURL(lastBlobUrlRef.current); lastBlobUrlRef.current = null; }
      if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current = null; }
    };
  }, []);

  // Titles/tooltips
  const titleText =
    status === 'connected'  ? `Video: Connected • ${pingMs ?? '—'} ms` :
    status === 'no_camera'  ? `Video: No camera`                          :
    status === 'connecting' ? `Video: Connecting…`                        :
                              `Video: Disconnected`;

  return (
    <div
      ref={containerRef}
      className={`relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden ${className}`}
    >
      {/* Header */}
      <div className="flex items-center justify-between px-3 py-2 bg-black/40 backdrop-blur border-b border-white/10">
        <div className="flex items-center gap-2 text-white/90">
          <StatusDot status={status} title={titleText} />
          <span className="text-sm font-semibold">{title}</span>
          <span className="text-xs text-white/70 ml-2">
            {pingMs != null ? `${pingMs} ms` : '— ms'} • {fps} fps
          </span>
        </div>
        <div className="flex items-center gap-1">
          <IconBtn onClick={handleTogglePlay} title={playing ? 'Pause' : 'Play'}>
            {playing ? '⏸' : '▶️'}
          </IconBtn>
          <IconBtn onClick={() => setFitMode(fitMode === 'cover' ? 'contain' : 'cover')} title={`Fit: ${fitMode}`}>
            {fitMode === 'cover' ? '◱' : '◰'}
          </IconBtn>
          <IconBtn onClick={() => setRotate((r) => ((r + 90) % 360) as 0 | 90 | 180 | 270)} title="Rotate 90°">↻</IconBtn>
          <IconBtn onClick={() => setFlipH(v => !v)} title="Flip H" active={flipH}>⇆</IconBtn>
          <IconBtn onClick={() => setFlipV(v => !v)} title="Flip V" active={flipV}>⥯</IconBtn>
          <IconBtn onClick={() => setFilter(f =>
            f === 'none' ? 'mono' : f === 'mono' ? 'contrast' : f === 'contrast' ? 'night' : 'none'
          )} title={`Filter: ${filter}`}>🎛️</IconBtn>
          <IconBtn onClick={handleSnapshotDownload} title="Snapshot PNG" disabled={!(status === 'connected' && playing)}>📸</IconBtn>
          <IconBtn
            onClick={recording ? stopRecording : startRecording}
            title={recording ? 'Stop recording' : 'Start recording'}
            active={recording}
            disabled={!recording && !(status === 'connected' && playing)}
          >
            {recording ? '⏺︎⏹' : '⏺'}
          </IconBtn>
          <IconBtn onClick={() => {
            const el = containerRef.current;
            if (!el) return;
            if (document.fullscreenElement) document.exitFullscreen().catch(()=>{});
            else el.requestFullscreen?.().catch(()=>{});
          }} title="Fullscreen">⛶</IconBtn>
        </div>
      </div>

      {/* Media area (16:9 default) */}
      <div className={`relative bg-black ${filterClass(filter)}`} style={{ paddingTop: '56.25%' }}>
        {playing ? (
          <Image
            ref={imgRef}
            key={IMG_SRC}
            src={IMG_SRC}
            alt="Live Camera"
            fill
            className={`${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
            style={mediaStyle}
            onError={handleImgError}
            onLoad={handleImgLoad}
            draggable={false}
          />
        ) : (
          snapshotUrl && (
            <Image
              src={snapshotUrl}
              alt="Last frame"
              fill
              className={`opacity-25 ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
              style={mediaStyle}
              draggable={false}
            />
          )
        )}

        {!playing && !snapshotUrl && (
          <div className="absolute inset-0 flex items-center justify-center text-white/70 text-sm">Paused</div>
        )}
        {status === 'disconnected' && playing && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-3">Not connected</div>
            <button
              type="button"
              className="px-2 py-1 rounded bg-black/40 hover:bg-black/55 text-white text-xs backdrop-blur border border-white/15 transition"
              onClick={() => { handlePlay(); }}
              title="Retry"
            >
              Retry
            </button>
          </div>
        )}
        {status === 'no_camera' && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-1">Camera missing</div>
            <div className="text-xs text-white/70">Check ribbon cable / device</div>
          </div>
        )}
      </div>

      {/* Hidden canvas used for snapshot/recording */}
      <canvas ref={canvasRef} className="hidden" />
    </div>
  );
};

export default CameraFrame;
