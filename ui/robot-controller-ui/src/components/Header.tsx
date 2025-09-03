/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/Header.tsx
# Summary:
#   App header with overall health + battery and per-service status pills.
#   - Uses profile-aware resolver (`netProfile`) for all endpoints
#   - WS services (movement/line/lighting): JSON ping→pong for latency
#   - Ultrasonic: marks alive on any message (no pong)
#   - Video: probes sibling `/health` (cheap 200/503) instead of touching MJPEG
#   - Colors match ServiceStatusBar (green=connected, amber=connecting, red=disconnected, sky=no_camera)
#   - Network extras (MVP):
#       • Live Link pill (click toggles Network Wizard; opens Quick Actions when shown)
#       • Quick Actions: Connect iPhone PAN, Connect Wi-Fi (inline form)
#       • Polls /api/net/summary every 8s (safe if backend missing)
*/

'use client';

import React, { useMemo, useState, useEffect, useCallback } from 'react';
import dynamic from 'next/dynamic';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';
import { net } from '@/utils/netProfile';

// Lazy (client-only) load for the wizard — matches your file location
const NetworkWizard = dynamic(() => import('@/components/NetworkWizard'), { ssr: false });

interface HeaderProps {
  batteryLevel: number;
}

/** Convert an MJPEG URL (.../video_feed[?]) to the sibling `/health`. */
function toHealthUrl(videoUrl?: string) {
  if (!videoUrl) return '';
  const m = videoUrl.match(/^(.*)\/video_feed(?:\?.*)?$/i);
  return m ? `${m[1]}/health` : `${videoUrl.replace(/\/$/, '')}/health`;
}

type HeaderState = ServiceStatus | HttpStatus; // allow 'no_camera' from http hook

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

  const content = (
    <>
      <Dot state={state} />
      <span className="text-white/90 truncate max-w-[18ch]">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </>
  );

  const title =
    titleOverride ?? `${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`;

  return isInteractive ? (
    <button
      type="button"
      className={base + interactive}
      onClick={onClick}
      title={title}
      aria-expanded={ariaExpanded}
      aria-controls={ariaControls}
    >
      {content}
    </button>
  ) : (
    <div className={base} title={title}>
      {content}
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
    try {
      setError(null);
      const res = await fetch('/api/net/summary', { method: 'GET' });
      if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
      const data = await res.json();
      setSummary(data);
    } catch (e: any) {
      // Gracefully degrade: keep existing summary; mark error
      setError(e?.message ?? 'fetch failed');
      if (summary == null) {
        setSummary({ linkType: 'unknown', online: false });
      }
    } finally {
      setLoading(false);
    }
  }, [summary]);

  useEffect(() => {
    let timer: ReturnType<typeof setInterval> | null = null;
    fetchOnce();
    timer = setInterval(fetchOnce, intervalMs);
    return () => { if (timer) clearInterval(timer); };
  }, [fetchOnce, intervalMs]);

  // Derive a header pill state
  const state: HeaderState =
    loading ? 'connecting'
    : summary?.online ? 'connected'
    : 'disconnected';

  // Nice label
  const label =
    summary?.linkType === 'wifi'      ? `Wi-Fi: ${summary?.ssid ?? '…'}` :
    summary?.linkType === 'pan'       ? `PAN: ${summary?.panDevice ?? 'iPhone'}` :
    summary?.linkType === 'eth'       ? 'Ethernet' :
    summary?.linkType === 'tailscale' ? 'Tailscale' :
    summary?.linkType === 'none'      ? 'Offline' :
                                        'Network';

  // Tooltip details
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
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify(body ?? {}),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data?.error || `${res.status} ${res.statusText}`);
    return data;
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
   Original Header + additions
   ============================ */

// Graceful wrappers: if a URL is empty, pretend disconnected (keeps UI stable)
const useMaybeWs = (url: string, opts: Parameters<typeof useWsStatus>[1]) =>
  url ? useWsStatus(url, opts) : { status: 'disconnected' as ServiceStatus, latency: null };

const useMaybeHttp = (url: string, opts: Parameters<typeof useHttpStatus>[1]) =>
  url ? useHttpStatus(url, opts) : { status: 'disconnected' as HttpStatus, latency: null };

const Header: React.FC<HeaderProps> = ({ batteryLevel }) => {
  const [showNetwork, setShowNetwork] = useState(false);
  const [showQuick, setShowQuick] = useState(false);
  const networkPanelId = 'network-wizard-panel';

  // Resolve all endpoints via the profile-aware helper (honors ?profile).
  const MOVE_URL  = net.ws.movement();
  const ULTRA_URL = net.ws.ultrasonic();
  const LINE_URL  = net.ws.line();
  const LIGHT_URL = net.ws.lighting();
  const VIDEO_URL = net.video();

  // Prefer probing /health for video to avoid pulling the MJPEG stream.
  const VIDEO_HEALTH = useMemo(() => toHealthUrl(VIDEO_URL), [VIDEO_URL]);

  // WS with pong → latency
  const move  = useMaybeWs(MOVE_URL,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useMaybeWs(LINE_URL,  { pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useMaybeWs(LIGHT_URL, { pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Streaming WS (any message marks alive)
  const ultra = useMaybeWs(ULTRA_URL, { treatAnyMessageAsAlive: true, pongTimeoutMs: 4000 });

  // HTTP /health mapping (503+placeholder → "no_camera")
  const video = useMaybeHttp(VIDEO_HEALTH, {
    intervalMs: 5000,
    timeoutMs: 2500,
    treat503AsConnecting: true,
  });

  // Net summary (MVP)
  const netSummary = useNetSummary(8000);

  const states = [move.status, ultra.status, line.status, light.status, video.status] as const;
  const upCount = states.filter((s) => s === 'connected').length;
  const allGood = upCount === states.length;

  const batteryClass =
    batteryLevel > 75 ? 'bg-green-500' :
    batteryLevel > 50 ? 'bg-yellow-500' :
    batteryLevel > 20 ? 'bg-blue-500 neon-blue' :
                        'bg-red-500';

  // Label with chevron to mimic "Network ▼/▲" while still showing the live link
  const liveLinkLabel = `${netSummary.label} ${showNetwork ? '▲' : '▼'}`;

  return (
    <div className="flex flex-col gap-2 bg-gray-800 text-white p-4 sticky top-0 z-10 shadow-md">
      <div className="flex justify-between items-center">
        <div className="text-lg font-bold">Robot Controller</div>

        {/* Overall status + battery */}
        <div className="flex items-center space-x-4">
          <div className="flex items-center text-sm">
            <span className="opacity-80">Status:</span>
            {allGood ? (
              <FaCheckCircle aria-label="All services reachable" className="text-green-500 ml-2" />
            ) : (
              <FaTimesCircle aria-label="Some services down" className="text-red-500 ml-2" />
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

      {/* Per-service pills */}
      <div className="flex flex-wrap gap-2 items-center">
        {/* Live Link pill (click toggles wizard; opens Quick Actions when showing) */}
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

        <Pill label="Movement"   state={move.status}   latency={move.latency} />
        <Pill label="Ultrasonic" state={ultra.status} />
        <Pill label="Line"       state={line.status}   latency={line.latency} />
        <Pill label="Lighting"   state={light.status}  latency={light.latency} />
        <Pill label="Video"      state={video.status}  latency={video.latency} />
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
