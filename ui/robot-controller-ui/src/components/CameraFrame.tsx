/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/CameraFrame.tsx
# Summary:
#   MJPEG camera frame with controls (play/pause, fit, rotate, flip, filters,
#   snapshot/record, fullscreen) and a status dot that matches the status bar.
#
#   Improvements:
#   - Always render <img> while playing → avoids "blank until connected" race.
#   - Robust load/error handling with cache-busting and backoff retries.
#   - Safer snapshot/record (CORS-aware). Cleanups for object URLs & intervals.
#   - Clear comments & small UX niceties (Retry button, keyboard-friendly).
#
#   Status mapping (via GET to /health):
#     • 200 → "connected" (green)
#     • 503 + { placeholder: true } → "no_camera" (blue)
#     • 503 otherwise → "connecting" (amber)
#     • network/CORS errors → "disconnected" (red)
*/

import React, { useEffect, useMemo, useRef, useState } from 'react';

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

/** Build the correct health URL for whatever `src` we were given.
 *  - If `src` is our same-origin MJPEG proxy (/api/video-proxy?...),
 *    switch to the health proxy (/api/video-health?...).
 *  - Otherwise, derive .../health from the upstream video URL. */
const buildHealthUrl = (srcUrl?: string) => {
  if (!srcUrl) return '';
  try {
    // Normalize to a URL for reliable parsing, even if `src` is relative
    const base = typeof window !== 'undefined' ? window.location.origin : 'http://localhost';
    const u = new URL(srcUrl, base);

    // If the pathname indicates our same-origin proxy, mirror its query to /api/video-health
    if (u.pathname.startsWith('/api/video-proxy')) {
      const qs = new URLSearchParams(u.search);
      qs.delete('b'); // drop our cache-buster
      const q = qs.toString();
      return `/api/video-health${q ? `?${q}` : ''}`;
    }

    // Direct upstream:
    // .../video_feed[?...] → .../health, else <base>/health
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = ''; // health doesn’t need stream query
      return u.toString();
    }
    u.pathname = (u.pathname.replace(/\/+$/,'') || '') + '/health';
    return u.toString();
  } catch {
    return '';
  }
};

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
  const [fps, setFps] = useState<number>(0);

  const [playing, setPlaying] = useState(true);
  const [fitMode, setFitMode] = useState<FitMode>(initialFit);
  const [rotate, setRotate] = useState<0 | 90 | 180 | 270>(0);
  const [flipH, setFlipH] = useState(false);
  const [flipV, setFlipV] = useState(false);
  const [filter, setFilter] = useState<FilterMode>('none');

  const [snapshotUrl, setSnapshotUrl] = useState<string | null>(null);
  const lastBlobUrlRef = useRef<string | null>(null); // revoke snapshot object URLs

  const recStreamRef = useRef<MediaStream | null>(null);
  const recorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);
  const [recording, setRecording] = useState(false);

  const rafRef = useRef<number | null>(null);
  const fpsCounterRef = useRef<{ last: number; frames: number }>({ last: performance.now(), frames: 0 });

  const retryTimerRef = useRef<number | null>(null);
  const backoffRef = useRef<number>(800); // ms; grows up to ~6s

  // ---- Stable cache-buster: only changes on play/retry/backoff ----
  const [buster, setBuster] = useState<number>(() => Date.now());
  const IMG_SRC = useMemo(() => {
    if (!src) return '';
    const sep = src.includes('?') ? '&' : '?';
    return `${src}${sep}b=${buster}`;
  }, [src, buster]);

  // ---------- Availability + latency via GET /health ----------
  useEffect(() => {
    let cancelled = false;
    const healthUrl = buildHealthUrl(src);

    const check = async () => {
      if (!healthUrl) {
        if (!cancelled) { setStatus('disconnected'); setPingMs(null); }
        return;
      }
      try {
        setStatus(s => (s === 'connected' || s === 'no_camera') ? s : 'connecting');
        const start = performance.now();
        const res = await fetch(healthUrl, { method: 'GET', cache: 'no-store' });
        const end = performance.now();
        if (cancelled) return;

        let body: any = null;
        try { body = await res.clone().json(); } catch {}

        const rtt = Math.max(0, Math.round(end - start));
        if (res.ok) {
          setStatus('connected'); setPingMs(rtt);
        } else if (res.status === 503 && body && body.placeholder === true) {
          setStatus('no_camera'); setPingMs(rtt);
        } else if (res.status === 503) {
          setStatus('connecting'); setPingMs(rtt);
        } else {
          setStatus('disconnected'); setPingMs(null);
        }
      } catch {
        if (!cancelled) { setStatus('disconnected'); setPingMs(null); }
      }
    };

    check();
    const t = window.setInterval(check, Math.max(2000, checkIntervalMs));
    return () => { cancelled = true; window.clearInterval(t); };
  }, [src, checkIntervalMs]);

  // ---------- FPS estimate (only while recording or snapshotting to canvas) ----------
  useEffect(() => {
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
        const now = performance.now();
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
    return () => { if (rafRef.current) cancelAnimationFrame(rafRef.current); };
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
      // Just bump buster → re-requests the stream once
      setBuster(Date.now());
      if (DEBUG) console.log('[CameraFrame] retrying MJPEG after', delay, 'ms');
      backoffRef.current = Math.min(6000, backoffRef.current * 1.6);
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
    clearRetryTimer();
  };

  const handlePlay = () => {
    setSnapshotUrl(null);
    setPlaying(true);
    resetBackoff();
    // force new request once
    setBuster(Date.now());
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
    } catch {
      console.warn('Snapshot blocked (likely CORS). Make sure the source is same-origin or set proper CORS headers.');
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
        const blob = new Blob(chunksRef.current, { type: mimeType || 'video/webm' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url; a.download = `recording-${new Date().toISOString().replace(/[:.]/g, '-')}.webm`;
        document.body.appendChild(a); a.click(); document.body.removeChild(a);
        setTimeout(() => URL.revokeObjectURL(url), 10000);
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

  // ---------- Image load/error → status + retries ----------
  const handleImgLoad = () => {
    setStatus('connected');
    resetBackoff();
    if (DEBUG) console.log('[CameraFrame] <img> load OK');
  };

  const handleImgError = () => {
    setStatus(prev => (prev === 'no_camera' ? prev : 'disconnected'));
    if (DEBUG) console.warn('[CameraFrame] <img> error');
    scheduleRetry();
  };

  // Cleanup timers, blob URLs, recorder on unmount
  useEffect(() => {
    return () => {
      clearRetryTimer();
      try { recorderRef.current?.stop(); } catch {}
      try { recStreamRef.current?.getTracks().forEach(t => t.stop()); } catch {}
      recorderRef.current = null; recStreamRef.current = null;
      if (lastBlobUrlRef.current) {
        URL.revokeObjectURL(lastBlobUrlRef.current);
        lastBlobUrlRef.current = null;
      }
    };
  }, []);

  // ---------- UI bits ----------
  const headerRight = (
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
  );

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
        {headerRight}
      </div>

      {/* Media area (16:9 default) */}
      <div className={`relative bg-black ${filterClass(filter)}`} style={{ paddingTop: '56.25%' }}>
        {/* Always render <img> while playing; use a stable URL that only
           changes when `buster` changes (play/retry/backoff). */}
        {playing ? (
          <img
            ref={imgRef}
            key={IMG_SRC}               /* new request only when buster changes */
            src={IMG_SRC}
            alt="Live Camera"
            className={`absolute inset-0 w-full h-full ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
            style={{ transform: mediaTransform, transformOrigin: 'center center' }}
            onError={handleImgError}
            onLoad={handleImgLoad}
            draggable={false}
          />
        ) : (
          snapshotUrl && (
            <img
              src={snapshotUrl}
              alt="Last frame"
              className={`absolute inset-0 w-full h-full opacity-25 ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
              style={{ transform: mediaTransform, transformOrigin: 'center center' }}
            />
          )
        )}

        {/* Overlay messages */}
        {!playing && !snapshotUrl && (
          <div className="absolute inset-0 flex items-center justify-center text-white/70 text-sm">Paused</div>
        )}
        {status === 'disconnected' && playing && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-3">Not connected</div>
            <IconBtn onClick={handlePlay} title="Retry">Retry</IconBtn>
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
