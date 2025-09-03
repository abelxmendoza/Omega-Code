/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/network/NetworkWizard.tsx
# Summary:
#   Network Wizard (standalone panel) to inspect & manage connectivity from the browser.
#   - Opens its own lightweight WebSocket to the Movement server (non-blocking for the rest of the app)
#   - Actions: Refresh info, Quick Actions (Connect PAN, fast Wi-Fi), Scan Wi-Fi, Join SSID, Forget SSID
#   - Shows current SSID/IP/iface and quick links (Movement endpoint, Video /health)
#   - Profile switcher (LAN / Tailscale / Hotspot) updates the UI ?profile param
#
# Backend (movement_ws_server.py) expected handlers:
#   { "command":"net-info" }                 -> { "type":"net-info", ssid, ip, iface, tailscaleIp?, ts }
#   { "command":"net-scan" }                 -> { "type":"net-scan", networks:[{ ssid, rssi, secure }] }
#   { "command":"net-join", ssid, pass }     -> { "type":"ack", status:"ok"|"error", action:"net-join", ... }
#   { "command":"net-forget", ssid }         -> { "type":"ack", status:"ok"|"error", action:"net-forget", ... }
#
# Notes:
#   • Header should import this as: dynamic(() => import('@/components/network/NetworkWizard'), { ssr:false })
*/

'use client';

import React from 'react';
import { net } from '@/utils/netProfile';

type WsState = 'connecting' | 'connected' | 'disconnected';

type NetInfo = {
  ssid?: string | null;
  ip?: string | null;
  iface?: string | null;
  tailscaleIp?: string | null;
  ts?: number;
};

type ScanEntry = { ssid: string; rssi?: number; secure?: boolean };

const Dot: React.FC<{ state: WsState }> = ({ state }) => {
  const color =
    state === 'connected'  ? 'bg-emerald-500' :
    state === 'connecting' ? 'bg-amber-400'  :
                              'bg-rose-500';
  return <span className={`inline-block w-2 h-2 rounded-full ${color}`} aria-hidden />;
};

const field =
  'px-2 py-1 bg-black/20 rounded border border-white/10 text-sm text-white placeholder:text-white/40 w-full';

function cx(...cls: (string | false | undefined | null)[]) {
  return cls.filter(Boolean).join(' ');
}

const NetworkWizard: React.FC = () => {
  // Profile-aware endpoints
  const WS_URL = React.useMemo(() => net.ws.movement() || '', []);
  const VIDEO_URL = React.useMemo(() => net.video() || '', []);

  const wsRef = React.useRef<WebSocket | null>(null);
  const retryRef = React.useRef<ReturnType<typeof setTimeout> | null>(null);

  const [wsState, setWsState] = React.useState<WsState>('disconnected');
  const [info, setInfo] = React.useState<NetInfo>({});
  const [scan, setScan] = React.useState<ScanEntry[]>([]);
  const [busy, setBusy] = React.useState(false);
  const [msg, setMsg] = React.useState<string>('');

  // Join form state (kept for full join/forget controls)
  const [ssid, setSsid] = React.useState('');
  const [pass, setPass] = React.useState('');

  const movementLink = React.useMemo(() => {
    if (!WS_URL) return '';
    // ws:// -> http://, wss:// -> https://
    return WS_URL.replace(/^ws(s)?:\/\//i, (_m, s) => `http${s ? 's' : ''}://`);
  }, [WS_URL]);

  const videoHealth = React.useMemo(() => {
    if (!VIDEO_URL) return '';
    const m = VIDEO_URL.match(/^(.*)\/video_feed(?:\?.*)?$/i);
    return m ? `${m[1]}/health` : `${VIDEO_URL.replace(/\/$/, '')}/health`;
  }, [VIDEO_URL]);

  // Safe sender (no-throw)
  const safeSend = React.useCallback((payload: any) => {
    try {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify(payload));
      }
    } catch {
      /* swallow */
    }
  }, []);

  // WebSocket lifecycle (self-contained; retries with backoff)
  React.useEffect(() => {
    if (!WS_URL) return;

    let cancelled = false;

    const open = (attempt = 0) => {
      if (cancelled) return;
      setWsState('connecting');

      try {
        const ws = new WebSocket(WS_URL);
        wsRef.current = ws;

        ws.onopen = () => {
          if (cancelled) return;
          setWsState('connected');
          // Pull current info immediately
          safeSend({ command: 'net-info' });
        };

        ws.onmessage = (ev) => {
          if (cancelled) return;
          try {
            const data = JSON.parse(ev.data);
            if (data?.type === 'net-info') {
              setInfo({
                ssid: data.ssid ?? null,
                ip: data.ip ?? data.ipv4 ?? null,
                iface: data.iface ?? null,
                tailscaleIp: data.tailscaleIp ?? null,
                ts: data.ts ?? Date.now(),
              });
              setMsg('');
            } else if (data?.type === 'net-scan' && Array.isArray(data.networks)) {
              setScan(data.networks);
              setMsg('');
            } else if (data?.type === 'ack') {
              if (data.status === 'ok') {
                setMsg(`✓ ${data.action || 'ok'}`);
                if (data.action === 'net-join' || data.action === 'net-forget') {
                  // refresh info post-change
                  safeSend({ command: 'net-info' });
                }
              } else {
                setMsg(`× ${data.error || 'error'}`);
              }
            }
          } catch {
            /* ignore parse errors */
          }
        };

        ws.onerror = () => {
          /* onclose will handle retry */
        };

        ws.onclose = () => {
          if (cancelled) return;
          setWsState('disconnected');
          const backoff = Math.min(1000 * 2 ** attempt, 10_000);
          retryRef.current = setTimeout(() => open(attempt + 1), backoff);
        };
      } catch {
        const backoff = Math.min(1000 * 2 ** attempt, 10_000);
        retryRef.current = setTimeout(() => open(attempt + 1), backoff);
      }
    };

    open(0);

    return () => {
      cancelled = true;
      if (retryRef.current) clearTimeout(retryRef.current);
      retryRef.current = null;
      try { wsRef.current?.close(); } catch {}
      wsRef.current = null;
    };
  }, [WS_URL, safeSend]);

  // --- Actions ---------------------------------------------------------------
  const refresh = () => safeSend({ command: 'net-info' });

  const doScan = async () => {
    setBusy(true);
    setMsg('Scanning…');
    safeSend({ command: 'net-scan' });
    // Response arrives via onmessage handler
    setBusy(false);
  };

  const join = async (ssidVal: string, passVal: string) => {
    if (!ssidVal) { setMsg('× SSID required'); return; }
    setBusy(true);
    safeSend({ command: 'net-join', ssid: ssidVal, pass: passVal });
    setBusy(false);
  };

  const forget = async (ssidVal: string) => {
    if (!ssidVal) { setMsg('× SSID required'); return; }
    setBusy(true);
    safeSend({ command: 'net-forget', ssid: ssidVal });
    setBusy(false);
  };

  const copy = async (txt?: string | null) => {
    if (!txt) return;
    try { await navigator.clipboard.writeText(txt); setMsg('✓ Copied'); }
    catch { setMsg('× Copy failed'); }
  };

  // Profile switcher updates ?profile and reloads (util reads it)
  const setProfile = (p: 'lan' | 'tailscale' | 'hotspot') => {
    const url = new URL(window.location.href);
    url.searchParams.set('profile', p);
    window.location.href = url.toString();
  };

  return (
    <div className="rounded-lg bg-zinc-800 p-3 text-white space-y-3 border border-white/10">
      <div className="flex items-center justify-between">
        <div className="font-semibold">Network Wizard</div>
        <div className="flex items-center gap-2 text-xs">
          <Dot state={wsState} />
          <span className="text-white/70">{wsState}</span>
          <button
            onClick={refresh}
            className="ml-2 px-2 py-1 rounded bg-black/30 hover:bg-black/40 text-xs border border-white/10"
            title="Refresh info"
          >
            Refresh
          </button>
        </div>
      </div>

      {/* Quick actions: PAN connect + fast Wi-Fi join */}
      <QuickActionsRow setMsg={setMsg} onAfterAction={refresh} />

      {/* Current info */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-2">
        <div className="bg-black/20 p-2 rounded border border-white/10">
          <div className="text-[11px] text-white/60">SSID</div>
          <div className="text-sm break-all">{info.ssid || '—'}</div>
        </div>
        <div className="bg-black/20 p-2 rounded border border-white/10">
          <div className="text-[11px] text-white/60">IPv4</div>
          <button
            className="text-sm underline decoration-dotted"
            onClick={() => copy(info.ip)} title="Copy IP"
          >
            {info.ip || '—'}
          </button>
        </div>
        <div className="bg-black/20 p-2 rounded border border-white/10">
          <div className="text-[11px] text-white/60">Iface</div>
          <div className="text-sm">{info.iface || '—'}</div>
        </div>
        <div className="bg-black/20 p-2 rounded border border-white/10">
          <div className="text-[11px] text-white/60">Tailscale</div>
          <button
            className="text-sm underline decoration-dotted"
            onClick={() => copy(info.tailscaleIp)} title="Copy Tailscale IP"
          >
            {info.tailscaleIp || '—'}
          </button>
        </div>
      </div>

      {/* Endpoints */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-2">
        <a
          className="block px-3 py-2 rounded bg-black/20 hover:bg-black/30 text-sm border border-white/10 truncate"
          href={movementLink} target="_blank" rel="noreferrer"
          title={movementLink || 'Movement endpoint'}
        >
          Movement: {movementLink || '—'}
        </a>
        <a
          className="block px-3 py-2 rounded bg-black/20 hover:bg-black/30 text-sm border border-white/10 truncate"
          href={videoHealth} target="_blank" rel="noreferrer"
          title={videoHealth || 'Video /health'}
        >
          Video /health: {videoHealth || '—'}
        </a>
      </div>

      {/* Profile switcher */}
      <div className="flex flex-wrap items-center gap-2">
        <div className="text-sm text-white/70">Profile:</div>
        <button onClick={() => setProfile('lan')} className="px-3 py-1 rounded bg-blue-600 hover:bg-blue-500 text-sm">
          LAN
        </button>
        <button onClick={() => setProfile('tailscale')} className="px-3 py-1 rounded bg-violet-600 hover:bg-violet-500 text-sm">
          Tailscale
        </button>
        <button onClick={() => setProfile('hotspot')} className="px-3 py-1 rounded bg-amber-600 hover:bg-amber-500 text-sm">
          Hotspot
        </button>
        <div className="text-xs text-white/50 ml-1">
          (updates ?profile and reloads)
        </div>
      </div>

      {/* Join / Forget (full controls) */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-2">
        <input
          className={field}
          placeholder="SSID"
          value={ssid}
          onChange={(e) => setSsid(e.target.value)}
          autoCapitalize="none" autoCorrect="off" spellCheck={false}
        />
        <input
          className={field}
          placeholder="Password"
          value={pass}
          onChange={(e) => setPass(e.target.value)}
          type="password"
        />
        <div className="flex gap-2">
          <button
            onClick={() => join(ssid.trim(), pass)}
            disabled={busy || !ssid}
            className={`px-3 py-1 rounded text-sm ${busy ? 'bg-emerald-900' : 'bg-emerald-600 hover:bg-emerald-500'}`}
            title="Join network"
          >
            Join
          </button>
          <button
            onClick={() => forget(ssid.trim())}
            disabled={busy || !ssid}
            className={`px-3 py-1 rounded text-sm ${busy ? 'bg-rose-900' : 'bg-rose-600 hover:bg-rose-500'}`}
            title="Forget network"
          >
            Forget
          </button>
        </div>
      </div>

      {/* Scan list */}
      <div className="bg-black/15 p-2 rounded border border-white/10">
        <div className="flex items-center justify-between mb-2">
          <div className="text-sm font-semibold">Nearby Networks</div>
          <button
            onClick={doScan}
            disabled={busy}
            className={`px-2 py-1 rounded text-xs ${busy ? 'bg-zinc-700' : 'bg-zinc-600 hover:bg-zinc-500'}`}
            title="Scan Wi-Fi"
          >
            Scan
          </button>
        </div>
        {scan.length === 0 ? (
          <div className="text-xs text-white/50">No results yet.</div>
        ) : (
          <ul className="space-y-1 max-h-44 overflow-auto pr-1">
            {scan.map((n, idx) => (
              <li key={`${n.ssid}-${idx}`} className="flex items-center justify-between gap-2">
                <div className="truncate">
                  <span className="text-sm">{n.ssid || '(hidden)'}</span>
                  <span className="text-xs text-white/50 ml-2">
                    {n.secure ? '🔒' : '🔓'}{' '}
                    {typeof n.rssi === 'number' ? `${n.rssi} dBm` : ''}
                  </span>
                </div>
                <button
                  onClick={() => { setSsid(n.ssid); }}
                  className="px-2 py-1 rounded bg-blue-600 hover:bg-blue-500 text-xs shrink-0"
                  title="Use this SSID"
                >
                  Use
                </button>
              </li>
            ))}
          </ul>
        )}
      </div>

      {/* Status line */}
      {msg && <div className="text-xs text-white/70">{msg}</div>}

      {/* Hint if server lacks handlers */}
      {wsState === 'connected' && !info.ip && !info.ssid && (
        <div className="text-[11px] text-amber-300">
          Heads up: backend may not implement net-* yet. Add handlers for
          <code className="mx-1">net-info</code>,
          <code className="mx-1">net-scan</code>,
          <code className="mx-1">net-join</code>,
          <code className="mx-1">net-forget</code>.
        </div>
      )}
    </div>
  );
};

export default NetworkWizard;

/* -----------------------------
   Quick actions (inline block)
   ----------------------------- */
const QuickActionsRow: React.FC<{
  setMsg: (m: string) => void;
  onAfterAction?: () => void;
}> = ({ setMsg, onAfterAction }) => {
  const [busy, setBusy] = React.useState<'pan' | 'wifi' | null>(null);
  const [ssid, setSsid] = React.useState('');
  const [pass, setPass] = React.useState('');

  const doPan = async () => {
    try {
      setBusy('pan');
      setMsg('');
      const res = await fetch('/api/net/pan/connect', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({}), // backend can infer default target (env/last used)
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(data?.error || `${res.status} ${res.statusText}`);
      setMsg('✓ PAN connect requested');
      onAfterAction?.();
    } catch (e: any) {
      setMsg(`× PAN failed: ${e?.message ?? e}`);
    } finally {
      setBusy(null);
    }
  };

  const doWifi = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!ssid || !pass) { setMsg('× Enter SSID and password'); return; }
    try {
      setBusy('wifi');
      setMsg('');
      const res = await fetch('/api/net/wifi/connect', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ ssid, psk: pass }),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(data?.error || `${res.status} ${res.statusText}`);
      setMsg(`✓ Wi-Fi connect requested: ${ssid}`);
      setSsid(''); setPass('');
      onAfterAction?.();
    } catch (e: any) {
      setMsg(`× Wi-Fi failed: ${e?.message ?? e}`);
    } finally {
      setBusy(null);
    }
  };

  return (
    <div className="bg-black/20 border border-white/10 rounded-md p-2">
      <div className="flex flex-wrap items-center gap-2">
        <button
          onClick={doPan}
          disabled={!!busy}
          title="Connect to iPhone PAN (Bluetooth)"
          className={cx(
            'text-xs px-2 py-1 rounded text-white',
            busy === 'pan' ? 'bg-slate-700' : 'bg-emerald-600 hover:bg-emerald-500'
          )}
        >
          Connect PAN
        </button>

        <form onSubmit={doWifi} className="flex flex-wrap items-center gap-2 grow">
          <input
            className="bg-zinc-900 border border-white/10 rounded px-2 py-1 text-xs min-w-[10rem] grow"
            placeholder="SSID"
            value={ssid}
            onChange={(e) => setSsid(e.target.value)}
            autoCapitalize="none" autoCorrect="off" spellCheck={false}
          />
          <input
            className="bg-zinc-900 border border-white/10 rounded px-2 py-1 text-xs min-w-[10rem] grow"
            placeholder="Wi-Fi password"
            type="password"
            value={pass}
            onChange={(e) => setPass(e.target.value)}
          />
          <button
            type="submit"
            disabled={!!busy}
            className={cx(
              'text-xs px-2 py-1 rounded text-white',
              busy === 'wifi' ? 'bg-slate-700' : 'bg-sky-600 hover:bg-sky-500'
            )}
            title="Connect to Wi-Fi"
          >
            Wi-Fi →
          </button>
        </form>
      </div>
    </div>
  );
};
