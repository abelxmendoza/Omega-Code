/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/Header.tsx
# Summary:
#   App header with overall health + battery and per-service status pills.
#   - Profile-aware endpoints (via netProfile)
#   - WS services: ping→pong for latency (movement/line/lighting)
#   - Ultrasonic: any message marks alive (no pong)
#   - Video: probes sibling `/health` (cheap 200/503) instead of MJPEG
#   - Live Link pill: toggles Network Wizard + exposes Quick Actions
#   - Polls /api/net/summary every 8s with timeout + abort + stale-guard
#   - Shows current profile and MOCK badge (when NEXT_PUBLIC_MOCK_WS=1|true)
#   - Defensive error handling throughout; never throws during render
*/

'use client';

import React, { useMemo, useState, useEffect, useCallback } from 'react';
import dynamic from 'next/dynamic';
import Link from 'next/link';
import Image from 'next/image';
import { CheckCircle, XCircle, Settings } from 'lucide-react';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';
import { net } from '@/utils/netProfile';

const NetworkWizard = dynamic(() => import('@/components/NetworkWizard'), { ssr: false });

interface HeaderProps {
  batteryLevel: number;
}

/* ----------------------------- utilities ------------------------------ */

type HeaderState = ServiceStatus | HttpStatus; // includes 'no_camera'

const truthy = (v: unknown) =>
  String(v ?? '').trim().toLowerCase() === '1' ||
  String(v ?? '').trim().toLowerCase() === 'true' ||
  String(v ?? '').trim().toLowerCase() === 'yes';

const MOCK_WS = truthy(process.env.NEXT_PUBLIC_MOCK_WS);

const getActiveProfile = (): 'lan' | 'tailscale' | 'local' => {
  // Only check URL parameters on client-side to avoid hydration errors
  if (typeof window === 'undefined') {
    // Server-side: always use environment variable or default
    const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
    return (['lan', 'tailscale', 'local'].includes(env) ? env : 'local') as any;
  }
  
  // Client-side: check URL override first, then environment
  try {
    const q = new URL(window.location.href).searchParams.get('profile');
    if (q && ['lan', 'tailscale', 'local'].includes(q)) return q as any;
  } catch {}
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'].includes(env) ? env : 'local') as any;
};

/** Convert an MJPEG URL (.../video_feed[?]) to sibling `/health`. */
function toHealthUrl(videoUrl?: string) {
  if (!videoUrl) return '';
  try {
    const u = new URL(videoUrl);
    if (/\/video_feed\/?$/i.test(u.pathname)) {
      u.pathname = u.pathname.replace(/\/video_feed\/?$/i, '/health');
      u.search = '';
    } else {
      u.pathname = (u.pathname.replace(/\/+$/, '') || '') + '/health';
    }
    return u.toString();
  } catch {
    // Fallback: string replace
    return videoUrl.replace(/\/video_feed(?:\?.*)?$/i, '').replace(/\/+$/, '') + '/health';
  }
}

const Dot: React.FC<{ state: HeaderState }> = ({ state }) => {
  const color =
    state === 'connected'   ? 'bg-emerald-500' :
    state === 'connecting'  ? 'bg-amber-400'  :
    state === 'no_camera'   ? 'bg-sky-500'    :
                              'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} aria-hidden />;
};

type PillProps = {
  label: string;
  state: HeaderState;
  latency?: number | null;
  onClick?: () => void;
  isInteractive?: boolean;
  ariaExpanded?: boolean;
  ariaControls?: string;
  titleOverride?: string;
};

const Pill: React.FC<PillProps> = ({
  label, state, latency, onClick, isInteractive, ariaExpanded, ariaControls, titleOverride,
}) => {
  const glyph =
    state === 'connected'  ? '✓' :
    state === 'connecting' ? '…' :
    state === 'no_camera'  ? '○' : '×';
  const glyphClass =
    state === 'connected'  ? 'text-emerald-400' :
    state === 'connecting' ? 'text-amber-300'  :
    state === 'no_camera'  ? 'text-sky-300'    : 'text-rose-400';

  const base =
    'flex items-center gap-1.5 text-[11px] px-2 py-1 rounded-md bg-black/30 border border-white/10';
  const interactive = isInteractive
    ? ' cursor-pointer hover:bg-black/40 focus:outline-none focus:ring-2 focus:ring-white/20'
    : '';

  const title = titleOverride ?? `${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`;

  return isInteractive ? (
    <button
      type="button"
      className={base + interactive}
      onClick={onClick}
      title={title}
      aria-expanded={ariaExpanded}
      aria-controls={ariaControls}
    >
      <Dot state={state} />
      <span className="text-white/90 truncate max-w-[18ch]">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </button>
  ) : (
    <div className={base} title={title}>
      <Dot state={state} />
      <span className="text-white/90 truncate max-w-[18ch]">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </div>
  );
};

/* ============================
   Net summary (MVP poller)
   ============================ */
type LinkType = 'wifi' | 'pan' | 'eth' | 'tailscale' | 'none' | 'unknown';
type NetSummary = {
  linkType: LinkType;
  online?: boolean;
  ssid?: string;
  panDevice?: string;
  ifname?: string;
  ipv4?: string;
  ipv6?: string;
  rssi?: number;
  gateway?: string;
};

function useNetSummary(intervalMs = 8000) {
  const [summary, setSummary] = useState<NetSummary | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError]     = useState<string | null>(null);

  const fetchOnce = useCallback(async () => {
    // stale-request guard + timeout
    const ac = new AbortController();
    const t  = setTimeout(() => ac.abort(), 3000);
    try {
      setError(null);
      const res = await fetch('/api/net/summary', { method: 'GET', signal: ac.signal, cache: 'no-store' });
      const ok = res.ok;
      let data: any = null;
      try { data = await res.json(); } catch { data = null; }
      if (!ok) throw new Error(`${res.status} ${res.statusText}`);
      setSummary(data ?? { linkType: 'unknown', online: false });
    } catch (e: any) {
      // Keep last good summary; only inject a minimal one if we have none
      setError(e?.message ?? 'fetch failed');
      setSummary(prev => prev ?? { linkType: 'unknown', online: false });
    } finally {
      clearTimeout(t);
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    let timer: ReturnType<typeof setInterval> | null = null;
    let mounted = true;
    (async () => { if (mounted) await fetchOnce(); })();
    timer = setInterval(() => { void fetchOnce(); }, intervalMs);
    return () => { mounted = false; if (timer) clearInterval(timer); };
  }, [fetchOnce, intervalMs]);

  const state: HeaderState =
    loading ? 'connecting'
    : summary?.online ? 'connected'
    : 'disconnected';

  const label =
    summary?.linkType === 'wifi'      ? `Wi-Fi: ${summary?.ssid ?? '…'}` :
    summary?.linkType === 'pan'       ? `PAN: ${summary?.panDevice ?? 'iPhone'}` :
    summary?.linkType === 'eth'       ? 'Ethernet' :
    summary?.linkType === 'tailscale' ? 'Tailscale' :
    summary?.linkType === 'none'      ? 'Offline' :
                                        'Network';

  const title = [
    summary?.ifname ? `if=${summary.ifname}` : '',
    summary?.ipv4   ? `ipv4=${summary.ipv4}` : '',
    summary?.gateway? `gw=${summary.gateway}`: '',
    summary?.rssi!=null && summary?.linkType==='wifi' ? `rssi=${summary.rssi}dBm` : ''
  ].filter(Boolean).join(' • ');

  return { summary, state, label, title, loading, error, refresh: fetchOnce };
}

/* ============================
   Quick Actions (PAN + Wi-Fi)
   ============================ */
function useNetActions() {
  const [busy, setBusy] = useState<string | null>(null);
  const [msg, setMsg]   = useState<string | null>(null);

  const postJson = async (url: string, body: any) => {
    const ac = new AbortController();
    const t  = setTimeout(() => ac.abort(), 5000);
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(body ?? {}),
        signal: ac.signal,
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(data?.error || `${res.status} ${res.statusText}`);
      return data;
    } finally {
      clearTimeout(t);
    }
  };

  const connectPan = async (macOrName?: string) => {
    setMsg(null); setBusy('pan');
    try {
      const data = await postJson('/api/net/pan/connect', { macOrName });
      setMsg(data?.message || 'Requested PAN connect');
    } catch (e: any) {
      setMsg(`PAN error: ${e?.message ?? 'failed'}`);
    } finally {
      setBusy(null);
    }
  };

  const connectWifi = async (ssid: string, psk: string) => {
    setMsg(null); setBusy('wifi');
    try {
      const data = await postJson('/api/net/wifi/connect', { ssid, psk });
      setMsg(data?.message || `Requested Wi-Fi connect to "${ssid}"`);
    } catch (e: any) {
      setMsg(`Wi-Fi error: ${e?.message ?? 'failed'}`);
    } finally {
      setBusy(null);
    }
  };

  return { busy, msg, connectPan, connectWifi, clearMsg: () => setMsg(null) };
}

const QuickActions: React.FC<{
  onDone?: () => void;
  onRefresh?: () => void;
}> = ({ onDone, onRefresh }) => {
  const { busy, msg, connectPan, connectWifi, clearMsg } = useNetActions();
  const [showWifi, setShowWifi] = useState(false);
  const [ssid, setSsid] = useState('');
  const [psk,  setPsk]  = useState('');

  return (
    <div className="w-full bg-black/20 rounded-md border border-white/10 p-2">
      <div className="flex flex-wrap gap-2 items-center">
        <button
          className={`px-3 py-1.5 rounded text-sm text-white ${
            busy === 'pan' ? 'bg-amber-600' : 'bg-indigo-600 hover:bg-indigo-700'
          }`}
          onClick={() => connectPan()}
          disabled={!!busy}
          title="Connect robot to iPhone Personal Hotspot (Bluetooth PAN)"
        >
          Connect iPhone PAN
        </button>

        <button
          className={`px-3 py-1.5 rounded text-sm text-white ${
            showWifi ? 'bg-sky-700' : 'bg-sky-600 hover:bg-sky-700'
          }`}
          onClick={() => setShowWifi(v => !v)}
          disabled={!!busy}
          title="Show Wi-Fi connect form"
        >
          Wi-Fi…
        </button>

        <button
          className="px-3 py-1.5 rounded text-sm text-white bg-zinc-700 hover:bg-zinc-600"
          onClick={onRefresh}
          disabled={!!busy}
          title="Refresh network summary"
        >
          Refresh
        </button>

        {onDone && (
          <button
            className="ml-auto px-3 py-1.5 rounded text-sm text-white bg-zinc-700 hover:bg-zinc-600"
            onClick={onDone}
          >
            Close
          </button>
        )}
      </div>

      {showWifi && (
        <form
          className="mt-2 grid grid-cols-1 gap-2 sm:grid-cols-3"
          onSubmit={(e) => { e.preventDefault(); connectWifi(ssid, psk); }}
        >
          <input
            className="px-2 py-1.5 rounded bg-zinc-800 text-white border border-white/10"
            placeholder="SSID"
            value={ssid}
            onChange={(e) => setSsid(e.target.value)}
            required
            autoComplete="off"
          />
          <input
            className="px-2 py-1.5 rounded bg-zinc-800 text-white border border-white/10"
            placeholder="Password"
            value={psk}
            onChange={(e) => setPsk(e.target.value)}
            required
            autoComplete="off"
            type="password"
          />
          <button
            type="submit"
            className={`px-3 py-1.5 rounded text-sm text-white ${
              busy === 'wifi' ? 'bg-amber-600' : 'bg-green-600 hover:bg-green-700'
            }`}
            disabled={!!busy}
            title="Send Wi-Fi connect request"
          >
            Connect Wi-Fi
          </button>
        </form>
      )}

      {msg && (
        <div className="mt-2 text-xs text-white/80">
          {msg}{' '}
          <button className="underline opacity-70 hover:opacity-100" onClick={clearMsg}>
            dismiss
          </button>
        </div>
      )}
    </div>
  );
};

/* ============================
   Header
   ============================ */

const Header: React.FC<HeaderProps> = ({ batteryLevel }) => {
  const [showNetwork, setShowNetwork] = useState(false);
  const [showQuick, setShowQuick] = useState(false);
  const [showServers, setShowServers] = useState(true);
  const networkPanelId = 'network-wizard-panel';

  // Resolve endpoints defensively (never throw during render)
  const { MOVE_URL, ULTRA_URL, LINE_URL, LIGHT_URL, VIDEO_URL } = useMemo(() => {
    try {
      return {
        MOVE_URL:  net.ws.movement()  || '',
        ULTRA_URL: net.ws.ultrasonic()|| '',
        LINE_URL:  net.ws.line()      || '',
        LIGHT_URL: net.ws.lighting()  || '',
        VIDEO_URL: net.video()        || '',
      };
    } catch {
      return { MOVE_URL:'', ULTRA_URL:'', LINE_URL:'', LIGHT_URL:'', VIDEO_URL:'' };
    }
  }, []);

  const VIDEO_HEALTH = useMemo(() => toHealthUrl(VIDEO_URL), [VIDEO_URL]);

  // WS with pong → latency
  const move  = useWsStatus(MOVE_URL,  { enabled: !!MOVE_URL,  pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useWsStatus(LINE_URL,  { enabled: !!LINE_URL,  pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useWsStatus(LIGHT_URL, { enabled: !!LIGHT_URL, pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Streaming WS (any message → alive)
  const ultra = useWsStatus(ULTRA_URL, {
    enabled: !!ULTRA_URL,
    treatAnyMessageAsAlive: true,
    pongTimeoutMs: 4000,
  });

  // HTTP /health (503+placeholder → "no_camera")
  const video = useHttpStatus(VIDEO_HEALTH, {
    enabled: !!VIDEO_HEALTH,
    intervalMs: 5000,
    timeoutMs: 2500,
    treat503AsConnecting: true,
  });

  // Net summary
  const netSummary = useNetSummary(8000);

  const states = [move.status, ultra.status, line.status, light.status, video.status] as const;
  const upCount = states.filter((s) => s === 'connected').length;
  const allGood = upCount === states.length;

  const batteryClass =
    batteryLevel > 75 ? 'bg-green-500' :
    batteryLevel > 50 ? 'bg-yellow-500' :
    batteryLevel > 20 ? 'bg-blue-500 neon-blue' :
                        'bg-red-500';

  const liveLinkLabel = `${netSummary.label} ${showNetwork ? '▲' : '▼'}`;
  const profile = getActiveProfile();

  return (
    <div className="flex flex-col gap-2 bg-gray-800 text-white p-4 sticky top-0 z-10 shadow-md">
      <div className="flex justify-between items-center">
        <div className="text-lg font-bold flex items-center gap-2">
          <Image
            src="/image/README/omegatechlogopro-noBackground.png"
            alt="Omega Tech Logo"
            width={40}
            height={40}
            className="h-10 w-auto object-contain"
          />
          Robot Controller
          {/* Profile + MOCK badges (no hostnames = no leaks) */}
          <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 border border-white/15">
            {profile.toUpperCase()}
          </span>
          {MOCK_WS && (
            <span className="text-[10px] px-1.5 py-0.5 rounded bg-amber-500/20 border border-amber-400/40 text-amber-100">
              MOCK
            </span>
          )}
          
          {/* Network Management Link */}
          <div className="text-[10px] px-1.5 py-0.5 rounded bg-blue-500/20 border border-blue-400/40 text-blue-100 hover:bg-blue-500/30 transition-colors">
            <Link 
              href="/network"
              title="Network Management"
            >
              <Settings className="w-3 h-3" />
            </Link>
          </div>
        </div>

        {/* Overall status + battery */}
        <div className="flex items-center space-x-4">
          <div className="flex items-center text-sm">
            <span className="opacity-80">Status:</span>
            {allGood ? (
              <CheckCircle aria-label="All services reachable" className="text-green-500 ml-2" />
            ) : (
              <XCircle aria-label="Some services down" className="text-red-500 ml-2" />
            )}
            <span className="ml-2 opacity-80">{upCount}/{states.length} online</span>
          </div>

          <div className="flex items-center text-sm">
            <span className="opacity-80">Battery:</span>
            <div className="ml-2 w-32 battery-container">
              <div className={`h-4 rounded ${batteryClass}`} style={{ width: `${batteryLevel}%` }} />
            </div>
            <span className="ml-2 opacity-80">{batteryLevel}%</span>
          </div>
        </div>
      </div>

      {/* Servers section with toggle */}
      <div className="flex flex-wrap gap-2 items-center">
        {/* Servers toggle button */}
        <button
          type="button"
          className="flex items-center gap-1.5 text-[11px] px-2 py-1 rounded-md bg-black/30 border border-white/10 cursor-pointer hover:bg-black/40 focus:outline-none focus:ring-2 focus:ring-white/20"
          onClick={() => setShowServers(!showServers)}
          title={`${showServers ? 'Hide' : 'Show'} server status pills`}
        >
          <span className="text-white/90">Servers</span>
          <span className="text-white/50">{showServers ? '▼' : '▶'}</span>
        </button>

        {/* Live Link pill */}
        <Pill
          label={liveLinkLabel}
          state={netSummary.state}
          titleOverride={netSummary.title || undefined}
          isInteractive
          onClick={() => {
            setShowNetwork(prev => {
              const next = !prev;
              if (next) setShowQuick(true);
              return next;
            });
          }}
          ariaExpanded={showNetwork}
          ariaControls={networkPanelId}
        />

        {/* Server status pills - collapsible */}
        {showServers && (
          <>
            <Pill label="Movement"   state={move.status}   latency={move.latency} />
            <Pill label="Ultrasonic" state={ultra.status} />
            <Pill label="Line"       state={line.status}   latency={line.latency} />
            <Pill label="Lighting"   state={light.status}  latency={light.latency} />
            <Pill label="Video"      state={video.status}  latency={video.latency} />
          </>
        )}
      </div>

      {/* Quick Actions row */}
      {showQuick && (
        <QuickActions
          onDone={() => setShowQuick(false)}
          onRefresh={netSummary.refresh}
        />
      )}

      {/* Embedded Network Wizard */}
      {showNetwork && (
        <div id={networkPanelId} className="mt-2">
          <NetworkWizard />
        </div>
      )}
    </div>
  );
};

export default Header;
