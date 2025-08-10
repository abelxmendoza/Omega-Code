/*
# File: /src/components/CameraFrame.tsx
# Summary:
Camera frame for MJPEG feeds with overlay controls & status:
- status dot + latency (HEAD ping) + FPS estimate
- play/pause, fullscreen
- snapshot (PNG), record (WebM*) — ONLY when connected & playing
- fit/cover, rotate, flip H/V
- optional filters (normal, mono, high-contrast, night)
- shows "Not connected" overlay when stream unreachable
*/

import React, { useEffect, useMemo, useRef, useState } from 'react';

type ServerStatus = 'connecting' | 'connected' | 'disconnected';
type FitMode = 'cover' | 'contain';
type FilterMode = 'none' | 'mono' | 'contrast' | 'night';

export interface CameraFrameProps {
  src: string;
  title?: string;
  className?: string;
  checkIntervalMs?: number;   // latency poll (HEAD) interval
  initialFit?: FitMode;
  showGrid?: boolean;
}

const StatusDot: React.FC<{ status: ServerStatus; title?: string }> = ({ status, title }) => {
  const color =
    status === 'connected' ? 'bg-emerald-500'
    : status === 'connecting' ? 'bg-slate-500'
    : 'bg-rose-500';
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

const filterClass = (mode: FilterMode) => {
  switch (mode) {
    case 'mono': return 'filter grayscale';
    case 'contrast': return 'filter contrast-200 brightness-90';
    case 'night': return 'filter contrast-150 saturate-150 hue-rotate-60';
    default: return '';
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

  const recStreamRef = useRef<MediaStream | null>(null);
  const recorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);
  const [recording, setRecording] = useState(false);
  const rafRef = useRef<number | null>(null);
  const fpsCounterRef = useRef<{ last: number; frames: number }>({ last: performance.now(), frames: 0 });

  // ---------- Server availability + latency via HEAD ----------
  useEffect(() => {
    let cancelled = false;
    const check = async () => {
      if (!src) {
        if (!cancelled) { setStatus('disconnected'); setPingMs(null); }
        return;
      }
      try {
        setStatus((s) => (s === 'connected' ? s : 'connecting'));
        const start = performance.now();
        const res = await fetch(src, { method: 'HEAD', cache: 'no-store', mode: 'cors' as RequestMode });
        const end = performance.now();
        if (cancelled) return;
        if (res.ok) {
          setStatus('connected');
          setPingMs(Math.max(0, Math.round(end - start)));
        } else {
          setStatus('disconnected'); setPingMs(null);
        }
      } catch {
        if (!cancelled) { setStatus('disconnected'); setPingMs(null); }
      }
    };
    check();
    const t = setInterval(check, Math.max(2000, checkIntervalMs));
    return () => { cancelled = true; clearInterval(t); };
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
          ctx.fillStyle = 'rgba(0,0,0,0.5)'; ctx.fillRect(8, 8, 210, 22);
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

  // ---------- Play/Pause ----------
  const ensureCanvasDraw = async () => {
    const img = imgRef.current, cvs = canvasRef.current;
    if (!img || !cvs) return;
    try {
      const w = img.naturalWidth || img.clientWidth || 640;
      const h = img.naturalHeight || img.clientHeight || 360;
      cvs.width = w; cvs.height = h;
      const ctx = cvs.getContext('2d'); if (!ctx) return;
      ctx.drawImage(img, 0, 0, w, h);
      ctx.fillStyle = 'rgba(0,0,0,0.5)'; ctx.fillRect(8, 8, 210, 22);
      ctx.fillStyle = '#fff'; ctx.font = '14px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas';
      ctx.fillText(new Date().toLocaleString(), 14, 24);
    } catch {}
  };

  const handlePause = async () => {
    await ensureCanvasDraw();
    setSnapshotUrl(canvasRef.current?.toDataURL('image/png') || null);
    if (imgRef.current) imgRef.current.src = '';
    setPlaying(false);
  };
  const handlePlay = () => {
    if (imgRef.current && src) {
      imgRef.current.src = src + (src.includes('?') ? '&' : '?') + 't=' + Date.now();
    }
    setSnapshotUrl(null);
    setPlaying(true);
  };
  const handleTogglePlay = () => (playing ? handlePause() : handlePlay());

  const online = status === 'connected';
  const canCapture = online && playing;

  // ---------- Snapshot PNG (only when connected & playing) ----------
  const handleSnapshotDownload = async () => {
    if (!canCapture) return;
    await ensureCanvasDraw();
    const cvs = canvasRef.current; if (!cvs) return;
    try {
      const dataUrl = cvs.toDataURL('image/png');
      const a = document.createElement('a');
      a.href = dataUrl;
      a.download = `snapshot-${new Date().toISOString().replace(/[:.]/g, '-')}.png`;
      document.body.appendChild(a); a.click(); document.body.removeChild(a);
    } catch {
      // If CORS blocks canvas export, don't attempt download — we only capture from live Pi feed.
      console.warn('Snapshot blocked (likely CORS).');
    }
  };

  // ---------- Record WebM via canvas.captureStream (only when connected & playing) ----------
  const startRecording = () => {
    if (!canCapture) return;
    const cvs = canvasRef.current; if (!cvs) return;
    try {
      const stream = cvs.captureStream(30);
      const mimeOptions = ['video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm'];
      let mimeType = ''; for (const m of mimeOptions) if (MediaRecorder.isTypeSupported(m)) { mimeType = m; break; }
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

  // ---------- View transform (CSS) ----------
  const mediaTransform = useMemo(() => {
    const r = rotate;
    const sx = flipH ? -1 : 1;
    const sy = flipV ? -1 : 1;
    return `rotate(${r}deg) scale(${sx}, ${sy})`;
  }, [rotate, flipH, flipV]);

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
      {/* Snapshot and Record are disabled unless we are connected AND playing */}
      <IconBtn onClick={handleSnapshotDownload} title="Snapshot PNG" disabled={!canCapture}>📸</IconBtn>
      <IconBtn
        onClick={recording ? stopRecording : startRecording}
        title={recording ? 'Stop recording' : 'Start recording'}
        active={recording}
        // If not currently recording, disable when we can't capture
        disabled={!recording && !canCapture}
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

  return (
    <div ref={containerRef} className={`relative bg-gray-900 rounded-lg shadow-md border border-white/10 overflow-hidden ${className}`}>
      {/* Header */}
      <div className="flex items-center justify-between px-3 py-2 bg-black/40 backdrop-blur border-b border-white/10">
        <div className="flex items-center gap-2 text-white/90">
          <StatusDot status={status} title={`Video: ${status}`} />
          <span className="text-sm font-semibold">{title}</span>
          <span className="text-xs text-white/70 ml-2">
            {pingMs != null ? `${pingMs} ms` : '— ms'} • {fps} fps
          </span>
        </div>
        {headerRight}
      </div>

      {/* Media area (16:9 default) */}
      <div className={`relative bg-black ${filterClass(filter)}`} style={{ paddingTop: '56.25%' }}>
        {/* Only render <img> when connected AND playing; avoids black box on failure */}
        {online && playing && (
          <img
            ref={imgRef}
            src={src ? src + (src.includes('?') ? '&' : '?') + 'b=' + Date.now() : ''}
            alt="Live Camera"
            className={`absolute inset-0 w-full h-full ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
            style={{ transform: mediaTransform, transformOrigin: 'center center' }}
            onError={() => setStatus('disconnected')}
            onLoad={() => setStatus('connected')}
          />
        )}

        {/* Show last snapshot faintly when offline */}
        {!online && snapshotUrl && (
          <img
            src={snapshotUrl}
            alt="Last frame"
            className={`absolute inset-0 w-full h-full opacity-25 ${fitMode === 'cover' ? 'object-cover' : 'object-contain'}`}
            style={{ transform: mediaTransform, transformOrigin: 'center center' }}
          />
        )}

        {/* Overlay messages */}
        {!online && (
          <div className="absolute inset-0 flex flex-col items-center justify-center text-white/85">
            <div className="text-sm mb-3">Not connected</div>
            <IconBtn onClick={handlePlay} title="Retry">Retry</IconBtn>
          </div>
        )}
        {online && !playing && !snapshotUrl && (
          <div className="absolute inset-0 flex items-center justify-center text-white/70 text-sm">Paused</div>
        )}
      </div>

      {/* Hidden canvas used for snapshot/recording */}
      <canvas ref={canvasRef} className="hidden" />
    </div>
  );
};

export default CameraFrame;
