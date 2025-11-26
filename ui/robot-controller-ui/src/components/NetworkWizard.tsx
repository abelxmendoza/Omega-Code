/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/network/NetworkWizard.tsx
# Summary:
#   Network Wizard (standalone panel) to inspect & manage connectivity from the browser.
#   - Opens its own lightweight WebSocket to the Movement server (non-blocking for the rest of the app)
#   - Actions: Refresh info, Quick Actions (Connect PAN, fast Wi-Fi), Scan Wi-Fi, Join SSID, Forget SSID
#   - Shows current SSID/IP/iface and quick links (Movement endpoint, Video /health)
#   - Profile switcher (LAN / Tailscale / Hotspot) updates the UI ?profile param
#   - NEW: Persists profile to localStorage and query (?profile) so it sticks across reloads.
#
# Backend (movement_ws_server.py) expected handlers:
#   { "command":"net-info" }                 -> { "type":"net-info", ssid, ip, iface, tailscaleIp?, ts }
#   { "command":"net-scan" }                 -> { "type":"net-scan", networks:[{ ssid, rssi, secure }] }
#   { "command":"net-join", ssid, pass }     -> { "type":"ack", status:"ok"|"error", action:"net-join", ... }
#   { "command":"net-forget", ssid }         -> { "type":"ack", status:"ok"|"error", action:"net-forget", ... }
#
# Notes:
#   • Header should import this as: dynamic(() => import('@/components/network/NetworkWizard'), { ssr:false })
#   • “Hotspot” maps to the app’s `local` profile internally (keeps environment consistent).
*/

'use client';

import React from 'react';
import { net } from '@/utils/netProfile';
import { setStoredProfile, type Profile } from '@/utils/profilePref';

type WsState = 'connecting' | 'connected' | 'disconnected';

type NetInfo = {
  ssid?: string | null;
  ip?: string | null;
  iface?: string | null;
  tailscaleIp?: string | null;
  ts?: number;
  panActive?: boolean;
  panDevice?: {
    deviceName?: string;
    deviceAlias?: string;
    batteryLevel?: string;
    macAddress?: string;
  };
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

  // Profile switcher: map UI labels → app profiles, persist, and reload.
  // "Hotspot" == app's "local" profile to stay consistent with env/options.
  const switchProfile = (uiChoice: 'lan' | 'tailscale' | 'hotspot') => {
    const mapped: Profile = uiChoice === 'hotspot' ? 'local' : uiChoice;
    setStoredProfile(mapped);
    const url = new URL(window.location.href);
    url.searchParams.set('profile', mapped);
    window.location.href = url.toString();
  };

  // Get current active profile for indicator
  const currentProfile = React.useMemo(() => {
    try {
      return net.profile();
    } catch {
      return 'local';
    }
  }, []);

  // Map profile to display info
  const profileInfo = React.useMemo(() => {
    switch (currentProfile) {
      case 'lan':
        return { label: 'LAN', color: 'bg-blue-500', description: 'Local Network' };
      case 'tailscale':
        return { label: 'Tailscale', color: 'bg-violet-500', description: 'Tailscale VPN' };
      case 'local':
        return { label: 'Hotspot', color: 'bg-amber-500', description: 'Local/Hotspot' };
      default:
        return { label: 'Unknown', color: 'bg-gray-500', description: 'Unknown Profile' };
    }
  }, [currentProfile]);

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

      {/* Current Network Profile Indicator */}
      <div className="bg-black/20 rounded border border-white/10 p-3 space-y-2">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${profileInfo.color}`} title={profileInfo.description}></div>
          <div className="text-sm">
            <span className="text-white/70">Active Profile: </span>
            <span className="font-medium text-white">{profileInfo.label}</span>
            <span className="text-white/50 ml-1">({profileInfo.description})</span>
          </div>
        </div>
        
        {/* Show current endpoints being used */}
        <div className="text-xs text-white/60 space-y-1">
          <div className="flex items-center gap-2">
            <span>Movement:</span>
            <span className="text-white/80 font-mono">{WS_URL || '—'}</span>
            <div className={`w-2 h-2 rounded-full ${
              wsState === 'connected' ? 'bg-green-400' : 
              wsState === 'connecting' ? 'bg-yellow-400' : 'bg-red-400'
            }`} title={`WebSocket: ${wsState}`}></div>
          </div>
          <div>Video: <span className="text-white/80 font-mono">{videoHealth || '—'}</span></div>
        </div>
      </div>

      {/* Quick actions: PAN connect + fast Wi-Fi join */}
      <QuickActionsRow setMsg={setMsg} onAfterAction={refresh} />

      {/* iPhone Device Info (when PAN is active) */}
      {info.panActive && info.panDevice && Object.keys(info.panDevice).length > 0 && (
        <div className="bg-amber-900/20 border border-amber-500/30 rounded p-3">
          <div className="flex items-center gap-2 mb-2">
            <span className="text-amber-400">📱</span>
            <span className="text-sm font-semibold text-amber-300">iPhone Connected</span>
          </div>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-2 text-xs">
            <div>
              <div className="text-amber-400/70">Device Name</div>
              <div className="text-white">{info.panDevice.deviceName || '—'}</div>
            </div>
            <div>
              <div className="text-amber-400/70">Alias</div>
              <div className="text-white">{info.panDevice.deviceAlias || '—'}</div>
            </div>
            <div>
              <div className="text-amber-400/70">Battery</div>
              <div className="text-white">{info.panDevice.batteryLevel || '—'}</div>
            </div>
            <div>
              <div className="text-amber-400/70">MAC Address</div>
              <button
                className="text-white underline decoration-dotted"
                onClick={() => copy(info.panDevice?.macAddress || '')} title="Copy MAC"
              >
                {info.panDevice.macAddress ? info.panDevice.macAddress.slice(-8) : '—'}
              </button>
            </div>
          </div>
        </div>
      )}

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
        <div className="text-sm text-white/70">Switch Profile:</div>
        <button 
          onClick={() => switchProfile('lan')} 
          className={`px-3 py-1 rounded text-sm transition-all ${
            currentProfile === 'lan' 
              ? 'bg-blue-500 ring-2 ring-blue-300 text-white font-medium' 
              : 'bg-blue-600 hover:bg-blue-500 text-white/90'
          }`}
        >
          LAN
        </button>
        <button 
          onClick={() => switchProfile('tailscale')} 
          className={`px-3 py-1 rounded text-sm transition-all ${
            currentProfile === 'tailscale' 
              ? 'bg-violet-500 ring-2 ring-violet-300 text-white font-medium' 
              : 'bg-violet-600 hover:bg-violet-500 text-white/90'
          }`}
        >
          Tailscale
        </button>
        <button 
          onClick={() => switchProfile('hotspot')} 
          className={`px-3 py-1 rounded text-sm transition-all ${
            currentProfile === 'local' 
              ? 'bg-amber-500 ring-2 ring-amber-300 text-white font-medium' 
              : 'bg-amber-600 hover:bg-amber-500 text-white/90'
          }`}
        >
          Hotspot
        </button>
        <div className="text-xs text-white/50 ml-1">
          (persists to localStorage; updates ?profile and reloads)
        </div>
      </div>

      {/* Join / Forget (full controls) */}
      <div className="bg-black/20 border border-white/10 rounded-md p-3 space-y-3">
        <div className="flex items-center gap-2">
          <div className="text-sm font-semibold">Connect to Wi-Fi Network</div>
          <button
            onClick={doScan}
            disabled={busy}
            className={`px-2 py-1 rounded text-xs ${busy ? 'bg-zinc-700' : 'bg-blue-600 hover:bg-blue-500'}`}
            title="Scan for nearby Wi-Fi networks"
          >
            🔍 Scan Networks
          </button>
        </div>
        
        <div className="text-xs text-white/60 mb-2">
          <strong>💡 What is SSID?</strong> SSID is your Wi-Fi network name (the name you see when connecting your phone/laptop to Wi-Fi).
          Click &quot;Scan Networks&quot; above to see available networks, then click &quot;Use&quot; on any network to auto-fill it.
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-3 gap-2">
          <div>
            <label className="text-xs text-white/70 mb-1 block">Network Name (SSID)</label>
            <input
              className={field}
              placeholder="Select from list below or type network name"
              value={ssid}
              onChange={(e) => setSsid(e.target.value)}
              autoCapitalize="none" autoCorrect="off" spellCheck={false}
              title="Wi-Fi network name (or select from scanned list below)"
            />
          </div>
          <div>
            <label className="text-xs text-white/70 mb-1 block">Password</label>
            <input
              className={field}
              placeholder="Wi-Fi password (if required)"
              value={pass}
              onChange={(e) => setPass(e.target.value)}
              type="password"
              title="Wi-Fi password (leave empty for open networks)"
            />
          </div>
          <div className="flex gap-2 items-end">
            <button
              onClick={() => join(ssid.trim(), pass)}
              disabled={busy || !ssid}
              className={`px-3 py-2 rounded text-sm flex-1 ${busy ? 'bg-emerald-900' : 'bg-emerald-600 hover:bg-emerald-500'}`}
              title="Connect to this Wi-Fi network"
            >
              Join
            </button>
            <button
              onClick={() => forget(ssid.trim())}
              disabled={busy || !ssid}
              className={`px-3 py-2 rounded text-sm ${busy ? 'bg-rose-900' : 'bg-rose-600 hover:bg-rose-500'}`}
              title="Forget saved network"
            >
              Forget
            </button>
          </div>
        </div>
      </div>

      {/* Scan list */}
      <div className="bg-black/15 p-3 rounded border border-white/10">
        <div className="flex items-center justify-between mb-2">
          <div className="text-sm font-semibold">Nearby Wi-Fi Networks</div>
          <button
            onClick={doScan}
            disabled={busy}
            className={`px-3 py-1.5 rounded text-xs font-medium ${busy ? 'bg-zinc-700' : 'bg-blue-600 hover:bg-blue-500'}`}
            title="Scan for nearby Wi-Fi networks"
          >
            {busy ? 'Scanning...' : '🔍 Scan'}
          </button>
        </div>
        {scan.length === 0 ? (
          <div className="text-xs text-white/50 py-4 text-center">
            No networks found. Click &quot;Scan&quot; to search for nearby Wi-Fi networks.
          </div>
        ) : (
          <div className="space-y-1 max-h-64 overflow-y-auto pr-1">
            {scan.map((n, idx) => (
              <button
                key={`${n.ssid}-${idx}`}
                onClick={() => { 
                  setSsid(n.ssid); 
                  setMsg(`Selected: ${n.ssid}${n.secure ? ' (password required)' : ' (open network)'}`);
                }}
                className="w-full flex items-center justify-between gap-3 p-2 rounded bg-black/20 hover:bg-black/40 border border-white/10 hover:border-blue-500/50 transition-all text-left"
                title={`Click to select "${n.ssid}"${n.secure ? ' (requires password)' : ' (open network)'}`}
              >
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2">
                    <span className="text-sm font-medium truncate">{n.ssid || '(hidden)'}</span>
                    {n.secure ? (
                      <span className="text-xs text-amber-400" title="Password required">🔒</span>
                    ) : (
                      <span className="text-xs text-green-400" title="Open network (no password)">🔓</span>
                    )}
                  </div>
                  <div className="text-xs text-white/50 mt-0.5">
                    {typeof n.rssi === 'number' ? (
                      <span className={n.rssi > -60 ? 'text-green-400' : n.rssi > -75 ? 'text-yellow-400' : 'text-red-400'}>
                        Signal: {n.rssi} dBm {n.rssi > -60 ? '(Strong)' : n.rssi > -75 ? '(Good)' : '(Weak)'}
                      </span>
                    ) : (
                      <span>Signal strength unknown</span>
                    )}
                  </div>
                </div>
                <div className="text-xs text-blue-400 font-medium shrink-0">
                  Use →
                </div>
              </button>
            ))}
          </div>
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
          <div className="flex-1 min-w-[10rem]">
            <label className="text-xs text-white/70 mb-1 block">Network Name (SSID)</label>
            <input
              className="bg-zinc-900 border border-white/10 rounded px-2 py-1 text-xs w-full"
              placeholder="Select from list below or type network name"
              value={ssid}
              onChange={(e) => setSsid(e.target.value)}
              autoCapitalize="none" autoCorrect="off" spellCheck={false}
              title="Wi-Fi network name (select from scanned list or type manually)"
            />
          </div>
          <div className="flex-1 min-w-[10rem]">
            <label className="text-xs text-white/70 mb-1 block">Password</label>
            <input
              className="bg-zinc-900 border border-white/10 rounded px-2 py-1 text-xs w-full"
              placeholder="Wi-Fi password (if required)"
              type="password"
              value={pass}
              onChange={(e) => setPass(e.target.value)}
              title="Wi-Fi password (leave empty for open networks)"
            />
          </div>
          <div className="flex items-end">
            <button
              type="submit"
              disabled={!!busy}
              className={cx(
                'text-xs px-3 py-1 rounded text-white h-7',
                busy === 'wifi' ? 'bg-slate-700' : 'bg-sky-600 hover:bg-sky-500'
              )}
              title="Connect to Wi-Fi network"
            >
              Connect
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};
