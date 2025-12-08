/**
 * Omega Network Wizard Component
 * 
 * Comprehensive network management UI with:
 * - AP/Client mode switching
 * - WiFi scanning and connection
 * - Tailscale VPN status
 * - Hotspot quick connect
 * - AP fallback alerts
 * Integrates with backend REST API: /api/network/*
 */

'use client';

import React, { useState, useEffect, useCallback } from 'react';
import {
  Wifi,
  WifiOff,
  RefreshCw,
  CheckCircle,
  XCircle,
  AlertCircle,
  Loader2,
  Signal,
  Smartphone,
  Shield,
  AlertTriangle,
  Lock,
  Unlock,
  X,
} from 'lucide-react';
import { robotFetch } from '@/utils/network';

interface NetworkStatus {
  ok: boolean;
  mode: string;
  services: {
    hostapd: string;
    dnsmasq: string;
    dhcpcd: string;
    wpa_supplicant: string;
  };
  wlan0_ip?: string;
  ap_ssid?: string;
  ap_ip?: string;
  ssid?: string;
  signal?: number;
  last_updated?: string;
}

interface NetworkInfo {
  ok: boolean;
  mode: string;
  services: Record<string, { status: string; enabled: boolean }>;
  interfaces: {
    wlan0: {
      ip?: string;
      ssid?: string;
      status: string;
    };
  };
  ap_config?: {
    ssid?: string;
    ip?: string;
  };
  last_updated?: string;
}

interface WiFiNetwork {
  ssid: string;
  security: string;
  signal: number;
  frequency?: string;
}

interface TailscaleStatus {
  enabled: boolean;
  ip?: string;
  status: 'connected' | 'disconnected';
  hostname?: string;
}

interface Toast {
  id: string;
  message: string;
  type: 'success' | 'error' | 'info';
}

export default function OmegaNetworkWizard() {
  const [status, setStatus] = useState<NetworkStatus | null>(null);
  const [info, setInfo] = useState<NetworkInfo | null>(null);
  const [tailscaleStatus, setTailscaleStatus] = useState<TailscaleStatus | null>(null);
  const [wifiNetworks, setWifiNetworks] = useState<WiFiNetwork[]>([]);
  const [loading, setLoading] = useState(false);
  const [scanning, setScanning] = useState(false);
  const [switching, setSwitching] = useState(false);
  const [connecting, setConnecting] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [toasts, setToasts] = useState<Toast[]>([]);
  const [connectDialogOpen, setConnectDialogOpen] = useState(false);
  const [selectedNetwork, setSelectedNetwork] = useState<WiFiNetwork | null>(null);
  const [password, setPassword] = useState('');
  const [hotspotSSID, setHotspotSSID] = useState('');
  const [hotspotPassword, setHotspotPassword] = useState('');
  const [apFallbackActive, setApFallbackActive] = useState(false);

  // Toast management
  const showToast = useCallback((message: string, type: 'success' | 'error' | 'info' = 'info') => {
    const id = Date.now().toString();
    setToasts((prev) => [...prev, { id, message, type }]);
    setTimeout(() => {
      setToasts((prev) => prev.filter((t) => t.id !== id));
    }, 5000);
  }, []);

  const fetchStatus = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await robotFetch('/api/network');
      if (response.offline) {
        setError('Robot backend unavailable');
        setLoading(false);
        return;
      }
      const data = await response.json();
      if (data.ok !== false) {
        setStatus({
          mode: data.mode || 'unknown',
          wlan0_ip: data.ip,
          ap_ssid: data.ap_config?.ssid,
          ap_ip: data.ap_config?.ip,
          ssid: data.ssid,
          signal: data.rssi,
          services: Object.fromEntries(
            Object.entries(data.services_running || {}).map(([k, v]: [string, any]) => [
              k,
              typeof v === 'object' ? v.status : v
            ])
          ),
          last_updated: data.last_updated,
        });

        // Check if AP mode is active (fallback scenario)
        if (data.mode === 'ap' && data.ap_config?.ssid === 'Omega1-AP') {
          setApFallbackActive(true);
        } else {
          setApFallbackActive(false);
        }
      } else {
        setError(data.error || 'Failed to fetch network status');
      }
    } catch (err) {
      setError(`Error: ${err instanceof Error ? err.message : 'Unknown error'}`);
    } finally {
      setLoading(false);
    }
  }, []);

  const fetchInfo = useCallback(async () => {
    try {
      const response = await robotFetch('/api/network');
      if (response.offline) {
        return;
      }
      const data = await response.json();
      if (data.ok !== false) {
        setInfo({
          ok: true,
          mode: data.mode,
          services: data.services_running || {},
          interfaces: {
            wlan0: {
              ip: data.ip,
              ssid: data.ssid,
              status: data.interface === 'wlan0' ? 'up' : 'down',
            },
          },
          ap_config: data.ap_config,
          last_updated: data.last_updated,
        });
      }
    } catch (err) {
      console.error('Failed to fetch network info:', err);
    }
  }, []);

  const fetchTailscaleStatus = useCallback(async () => {
    try {
      const response = await robotFetch('/api/network/tailscale');
      if (response.offline) {
        return;
      }
      const data = await response.json();
      if (data.ok !== false) {
        setTailscaleStatus({
          enabled: data.enabled || false,
          ip: data.ip,
          status: data.status || 'disconnected',
          hostname: data.hostname,
        });
      }
    } catch (err) {
      console.error('Failed to fetch Tailscale status:', err);
    }
  }, []);

  const scanWiFi = useCallback(async () => {
    setScanning(true);
    try {
      const response = await robotFetch('/api/network/wifi/scan');
      if (response.offline) {
        showToast('Robot backend unavailable', 'error');
        setScanning(false);
        return;
      }
      const data = await response.json();
      if (data.ok && data.networks) {
        const sorted = [...data.networks].sort((a, b) => b.signal - a.signal);
        setWifiNetworks(sorted);
        showToast(`Found ${data.count} networks`, 'success');
      } else {
        showToast('Failed to scan networks', 'error');
      }
    } catch (err) {
      showToast('Error scanning networks', 'error');
    } finally {
      setScanning(false);
    }
  }, [showToast]);

  useEffect(() => {
    fetchStatus();
    fetchInfo();
    fetchTailscaleStatus();
    const interval = setInterval(() => {
      fetchStatus();
      fetchInfo();
      fetchTailscaleStatus();
    }, 10000); // Refresh every 10 seconds
    return () => clearInterval(interval);
  }, [fetchStatus, fetchInfo, fetchTailscaleStatus]);

  const switchMode = useCallback(async (mode: 'ap' | 'client') => {
    setSwitching(true);
    setError(null);
    setSuccess(null);

    try {
      const response = await robotFetch('/api/network/mode', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ mode }),
      });

      if (response.offline) {
        setError('Robot backend unavailable');
        setSwitching(false);
        return;
      }

      const data = await response.json();

      if (data.ok) {
        const message = data.message || `Switched to ${mode.toUpperCase()} mode`;
        setSuccess(message);
        showToast(message, 'success');
        setTimeout(() => {
          fetchStatus();
          fetchInfo();
        }, 2000);
      } else {
        const errorMsg = data.detail || 'Failed to switch network mode';
        setError(errorMsg);
        showToast(errorMsg, 'error');
      }
    } catch (err) {
      const errorMsg = `Error: ${err instanceof Error ? err.message : 'Unknown error'}`;
      setError(errorMsg);
      showToast(errorMsg, 'error');
    } finally {
      setSwitching(false);
    }
  }, [fetchStatus, fetchInfo, showToast]);

  const connectToWiFi = useCallback(async (ssid: string, pwd: string) => {
    setConnecting(ssid);
    try {
      const response = await robotFetch('/api/network/wifi/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ssid, password: pwd }),
      });

      if (response.offline) {
        showToast('Robot backend unavailable', 'error');
        setConnecting(null);
        return;
      }

      const data = await response.json();
      if (data.ok) {
        showToast(`Connected to ${ssid}`, 'success');
        setConnectDialogOpen(false);
        setPassword('');
        setSelectedNetwork(null);
        setTimeout(() => {
          fetchStatus();
          fetchInfo();
        }, 2000);
      } else {
        showToast(data.error || data.detail || 'Failed to connect', 'error');
      }
    } catch (err) {
      showToast('Error connecting to network', 'error');
    } finally {
      setConnecting(null);
    }
  }, [fetchStatus, fetchInfo, showToast]);

  const connectToHotspot = useCallback(async () => {
    if (!hotspotSSID.trim()) {
      showToast('Please enter hotspot SSID', 'error');
      return;
    }
    await connectToWiFi(hotspotSSID.trim(), hotspotPassword);
  }, [hotspotSSID, hotspotPassword, connectToWiFi, showToast]);

  const restartTailscale = useCallback(async () => {
    try {
      const response = await robotFetch('/api/services/restart/tailscale', {
        method: 'POST',
      });
      if (response.offline) {
        showToast('Robot backend unavailable', 'error');
        return;
      }
      const data = await response.json();
      if (data.ok) {
        showToast('Tailscale restarted', 'success');
        setTimeout(() => {
          fetchTailscaleStatus();
        }, 2000);
      } else {
        showToast('Failed to restart Tailscale', 'error');
      }
    } catch (err) {
      showToast('Error restarting Tailscale', 'error');
    }
  }, [fetchTailscaleStatus, showToast]);

  const validateConfig = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await robotFetch('/api/network/validate', {
        method: 'POST',
      });

      if (response.offline) {
        setError('Robot backend unavailable');
        setLoading(false);
        return;
      }

      const data = await response.json();

      if (data.ok && data.valid) {
        setSuccess('Network configuration is valid');
        showToast('Network configuration is valid', 'success');
      } else {
        const failedChecks = Object.entries(data.results || {})
          .filter(([_, passed]) => !passed)
          .map(([check, _]) => check)
          .join(', ');
        const errorMsg = data.error || `Validation failed: ${failedChecks || 'Unknown error'}`;
        setError(errorMsg);
        showToast(errorMsg, 'error');
      }
    } catch (err) {
      const errorMsg = `Error: ${err instanceof Error ? err.message : 'Unknown error'}`;
      setError(errorMsg);
      showToast(errorMsg, 'error');
    } finally {
      setLoading(false);
    }
  }, [showToast]);

  const getServiceIcon = (serviceStatus: string) => {
    if (serviceStatus === 'active') {
      return <CheckCircle className="w-4 h-4 text-green-400" />;
    }
    return <XCircle className="w-4 h-4 text-red-400" />;
  };

  const getModeColor = (mode: string) => {
    if (mode === 'ap') return 'text-purple-400';
    if (mode === 'client') return 'text-blue-400';
    return 'text-gray-400';
  };

  const getSignalIcon = (signal?: number) => {
    if (!signal) return <Signal className="w-4 h-4 text-gray-400" />;
    if (signal >= 70) return <Signal className="w-4 h-4 text-green-400" />;
    if (signal >= 40) return <Signal className="w-4 h-4 text-yellow-400" />;
    return <Signal className="w-4 h-4 text-red-400" />;
  };

  const getSecurityIcon = (security: string) => {
    if (security === 'Open' || !security) {
      return <Unlock className="w-4 h-4 text-gray-400" />;
    }
    return <Lock className="w-4 h-4 text-blue-400" />;
  };

  return (
    <div className="bg-gray-900 border border-gray-700 rounded-lg p-6 shadow-lg max-w-4xl">
      {/* Toast Notifications */}
      <div className="fixed top-4 right-4 z-50 space-y-2">
        {toasts.map((toast) => (
          <div
            key={toast.id}
            className={`p-4 rounded-lg shadow-lg border flex items-center gap-3 min-w-[300px] animate-in slide-in-from-right ${
              toast.type === 'success'
                ? 'bg-green-900/90 border-green-500/50 text-green-100'
                : toast.type === 'error'
                ? 'bg-red-900/90 border-red-500/50 text-red-100'
                : 'bg-blue-900/90 border-blue-500/50 text-blue-100'
            }`}
          >
            {toast.type === 'success' && <CheckCircle className="w-5 h-5" />}
            {toast.type === 'error' && <XCircle className="w-5 h-5" />}
            {toast.type === 'info' && <AlertCircle className="w-5 h-5" />}
            <span className="flex-1">{toast.message}</span>
            <button
              onClick={() => setToasts((prev) => prev.filter((t) => t.id !== toast.id))}
              className="text-current opacity-70 hover:opacity-100"
            >
              <X className="w-4 h-4" />
            </button>
          </div>
        ))}
      </div>

      <div className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-3">
          <Wifi className="w-6 h-6 text-purple-400" />
          <h2 className="text-xl font-bold text-white">Omega Network Wizard</h2>
        </div>
        <button
          onClick={() => { fetchStatus(); fetchInfo(); fetchTailscaleStatus(); }}
          disabled={loading}
          className="p-2 rounded bg-gray-800 hover:bg-gray-700 text-white transition-colors disabled:opacity-50"
          title="Refresh status"
        >
          <RefreshCw className={`w-5 h-5 ${loading ? 'animate-spin' : ''}`} />
        </button>
      </div>

      {/* AP Fallback Alert */}
      {apFallbackActive && (
        <div className="mb-6 p-4 bg-yellow-900/30 border border-yellow-500/50 rounded-lg flex items-start gap-3">
          <AlertTriangle className="w-6 h-6 text-yellow-400 flex-shrink-0 mt-0.5" />
          <div className="flex-1">
            <h3 className="font-semibold text-yellow-200 mb-1">AP Fallback Mode Active</h3>
            <p className="text-sm text-yellow-300/80">
              Robot booted into AP mode due to WiFi connection failure. Connect to{' '}
              <span className="font-mono font-semibold">Omega1-AP</span> to configure WiFi or switch back to client mode.
            </p>
          </div>
        </div>
      )}

      {/* Status Messages */}
      {error && (
        <div className="mb-4 p-3 bg-red-900/30 border border-red-500/50 rounded flex items-center gap-2 text-red-200">
          <AlertCircle className="w-5 h-5" />
          <span>{error}</span>
        </div>
      )}

      {success && (
        <div className="mb-4 p-3 bg-green-900/30 border border-green-500/50 rounded flex items-center gap-2 text-green-200">
          <CheckCircle className="w-5 h-5" />
          <span>{success}</span>
        </div>
      )}

      {/* Current Status */}
      {status && (
        <div className="mb-6 space-y-4">
          <div className="bg-gray-800 rounded-lg p-4">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-lg font-semibold text-white">Current Status</h3>
              <span className={`px-3 py-1 rounded text-sm font-medium ${getModeColor(status.mode)} bg-gray-700`}>
                {status.mode.toUpperCase()} Mode
              </span>
            </div>

            {/* IP Address */}
            {status.wlan0_ip && (
              <div className="mb-2">
                <span className="text-gray-400 text-sm">IP Address:</span>
                <span className="ml-2 text-white font-mono">{status.wlan0_ip}</span>
              </div>
            )}

            {/* SSID and Signal */}
            {status.ssid && (
              <div className="mb-2 flex items-center gap-2">
                <span className="text-gray-400 text-sm">Connected to:</span>
                <span className="text-white font-mono">{status.ssid}</span>
                {status.signal !== undefined && (
                  <>
                    {getSignalIcon(status.signal)}
                    <span className="text-gray-400 text-sm">{status.signal}%</span>
                  </>
                )}
              </div>
            )}

            {/* AP Configuration */}
            {status.mode === 'ap' && status.ap_ssid && (
              <div className="mb-2 space-y-1">
                <div>
                  <span className="text-gray-400 text-sm">SSID:</span>
                  <span className="ml-2 text-white font-mono">{status.ap_ssid}</span>
                </div>
                {status.ap_ip && (
                  <div>
                    <span className="text-gray-400 text-sm">AP IP:</span>
                    <span className="ml-2 text-white font-mono">{status.ap_ip}</span>
                  </div>
                )}
                <div className="text-xs text-gray-500 mt-2">
                  Connect to <span className="font-mono text-purple-400">{status.ap_ssid}</span> and SSH to{' '}
                  <span className="font-mono text-purple-400">omega1@{status.ap_ip || '192.168.4.1'}</span>
                </div>
              </div>
            )}

            {/* Service Status */}
            <div className="mt-4 pt-4 border-t border-gray-700">
              <h4 className="text-sm font-semibold text-gray-400 mb-2">Services</h4>
              <div className="grid grid-cols-2 gap-2">
                {Object.entries(status.services).map(([service, serviceStatus]) => (
                  <div key={service} className="flex items-center gap-2 text-sm">
                    {getServiceIcon(serviceStatus)}
                    <span className="text-gray-300 capitalize">{service}</span>
                    <span className={`ml-auto text-xs ${serviceStatus === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                      {serviceStatus}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Mode Switching */}
      <div className="mb-6">
        <h3 className="text-lg font-semibold text-white mb-3">Switch Mode</h3>
        <div className="grid grid-cols-2 gap-3">
          <button
            onClick={() => switchMode('ap')}
            disabled={switching || status?.mode === 'ap'}
            className="p-4 rounded-lg bg-purple-900/30 border border-purple-500/50 hover:bg-purple-900/50 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <div className="flex items-center justify-center gap-2">
              {switching && status?.mode !== 'ap' ? (
                <Loader2 className="w-5 h-5 animate-spin text-purple-400" />
              ) : (
                <Wifi className="w-5 h-5 text-purple-400" />
              )}
              <span className="text-white font-medium">AP Mode</span>
            </div>
            <p className="text-xs text-gray-400 mt-2 text-center">
              Create Wi-Fi hotspot (Omega1-AP)
            </p>
          </button>

          <button
            onClick={() => switchMode('client')}
            disabled={switching || status?.mode === 'client'}
            className="p-4 rounded-lg bg-blue-900/30 border border-blue-500/50 hover:bg-blue-900/50 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <div className="flex items-center justify-center gap-2">
              {switching && status?.mode !== 'client' ? (
                <Loader2 className="w-5 h-5 animate-spin text-blue-400" />
              ) : (
                <WifiOff className="w-5 h-5 text-blue-400" />
              )}
              <span className="text-white font-medium">Client Mode</span>
            </div>
            <p className="text-xs text-gray-400 mt-2 text-center">
              Connect to existing Wi-Fi
            </p>
          </button>
        </div>
      </div>

      {/* WiFi Scanner */}
      <div className="mb-6">
        <div className="flex items-center justify-between mb-3">
          <h3 className="text-lg font-semibold text-white">WiFi Networks</h3>
          <button
            onClick={scanWiFi}
            disabled={scanning}
            className="px-4 py-2 rounded bg-gray-800 hover:bg-gray-700 text-white transition-colors disabled:opacity-50 text-sm flex items-center gap-2"
          >
            {scanning ? (
              <>
                <Loader2 className="w-4 h-4 animate-spin" />
                Scanning...
              </>
            ) : (
              <>
                <RefreshCw className="w-4 h-4" />
                Scan Networks
              </>
            )}
          </button>
        </div>
        {wifiNetworks.length === 0 ? (
          <div className="bg-gray-800 rounded-lg p-6 text-center text-gray-400">
            <Wifi className="w-12 h-12 mx-auto mb-2 opacity-50" />
            <p>No networks found. Click &quot;Scan Networks&quot; to search.</p>
          </div>
        ) : (
          <div className="bg-gray-800 rounded-lg p-4 space-y-2 max-h-[300px] overflow-y-auto">
            {wifiNetworks.map((network) => (
              <div
                key={network.ssid}
                className="flex items-center justify-between p-3 rounded-lg border border-gray-700 hover:bg-gray-700/50 cursor-pointer transition-colors"
                onClick={() => {
                  setSelectedNetwork(network);
                  setConnectDialogOpen(true);
                }}
              >
                <div className="flex items-center gap-3 flex-1">
                  {getSecurityIcon(network.security)}
                  <div className="flex-1 min-w-0">
                    <div className="font-medium text-white truncate">{network.ssid}</div>
                    <div className="text-xs text-gray-400">
                      {network.security || 'Open'} • {network.signal}%
                    </div>
                  </div>
                </div>
                <div className="flex items-center gap-2">
                  {getSignalIcon(network.signal)}
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      setSelectedNetwork(network);
                      setConnectDialogOpen(true);
                    }}
                    className="px-3 py-1 rounded bg-purple-900/30 border border-purple-500/50 hover:bg-purple-900/50 text-white text-sm transition-colors"
                  >
                    Connect
                  </button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Hotspot Quick Connect */}
      <div className="mb-6">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Smartphone className="w-5 h-5 text-purple-400" />
          Hotspot Quick Connect
        </h3>
        <div className="bg-gray-800 rounded-lg p-4 space-y-3">
          <div>
            <label className="text-sm text-gray-400 mb-1 block">Hotspot SSID</label>
            <input
              type="text"
              placeholder="iPhone's Name"
              value={hotspotSSID}
              onChange={(e) => setHotspotSSID(e.target.value)}
              className="w-full px-3 py-2 rounded bg-gray-700 border border-gray-600 text-white placeholder-gray-500 focus:outline-none focus:border-purple-500"
            />
          </div>
          <div>
            <label className="text-sm text-gray-400 mb-1 block">Password</label>
            <input
              type="password"
              placeholder="Hotspot password"
              value={hotspotPassword}
              onChange={(e) => setHotspotPassword(e.target.value)}
              className="w-full px-3 py-2 rounded bg-gray-700 border border-gray-600 text-white placeholder-gray-500 focus:outline-none focus:border-purple-500"
            />
          </div>
          <button
            onClick={connectToHotspot}
            disabled={!hotspotSSID.trim() || connecting === hotspotSSID}
            className="w-full px-4 py-2 rounded bg-purple-900/30 border border-purple-500/50 hover:bg-purple-900/50 text-white transition-colors disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2"
          >
            {connecting === hotspotSSID ? (
              <>
                <Loader2 className="w-4 h-4 animate-spin" />
                Connecting...
              </>
            ) : (
              <>
                <Smartphone className="w-4 h-4" />
                Connect to Hotspot
              </>
            )}
          </button>
        </div>
      </div>

      {/* Tailscale Panel */}
      <div className="mb-6">
        <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
          <Shield className="w-5 h-5 text-purple-400" />
          Tailscale VPN
        </h3>
        <div className="bg-gray-800 rounded-lg p-4">
          {tailscaleStatus ? (
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-400">Status</span>
                <span className={`px-3 py-1 rounded text-sm font-medium ${
                  tailscaleStatus.status === 'connected'
                    ? 'bg-green-900/30 border border-green-500/50 text-green-400'
                    : 'bg-gray-700 text-gray-400'
                }`}>
                  {tailscaleStatus.status === 'connected' ? (
                    <>
                      <CheckCircle className="w-3 h-3 inline mr-1" />
                      Connected
                    </>
                  ) : (
                    <>
                      <XCircle className="w-3 h-3 inline mr-1" />
                      Disconnected
                    </>
                  )}
                </span>
              </div>

              {tailscaleStatus.ip && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-400">Tailscale IP</span>
                  <span className="font-mono text-sm text-white">{tailscaleStatus.ip}</span>
                </div>
              )}

              {tailscaleStatus.hostname && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-gray-400">Hostname</span>
                  <span className="font-mono text-sm text-white">{tailscaleStatus.hostname}</span>
                </div>
              )}

              {!tailscaleStatus.enabled && (
                <div className="text-sm text-gray-400">
                  Tailscale is not installed or not enabled.
                </div>
              )}

              {tailscaleStatus.enabled && (
                <button
                  onClick={restartTailscale}
                  className="w-full px-4 py-2 rounded bg-gray-700 hover:bg-gray-600 text-white transition-colors flex items-center justify-center gap-2"
                >
                  <RefreshCw className="w-4 h-4" />
                  Restart Tailscale
                </button>
              )}
            </div>
          ) : (
            <div className="text-center py-4 text-gray-400">
              <Loader2 className="w-6 h-6 mx-auto mb-2 animate-spin" />
              <p>Loading Tailscale status...</p>
            </div>
          )}
        </div>
      </div>

      {/* Additional Info */}
      {info && (
        <div className="mb-6 bg-gray-800 rounded-lg p-4">
          <h3 className="text-sm font-semibold text-gray-400 mb-2">Network Details</h3>
          {info.interfaces.wlan0.ssid && (
            <div className="text-sm text-gray-300 mb-1">
              <span className="text-gray-400">Connected to:</span>{' '}
              <span className="font-mono text-white">{info.interfaces.wlan0.ssid}</span>
            </div>
          )}
          {info.last_updated && (
            <div className="text-xs text-gray-500 mt-2">
              Last updated: {new Date(info.last_updated).toLocaleString()}
            </div>
          )}
        </div>
      )}

      {/* Actions */}
      <div className="flex gap-3">
        <button
          onClick={validateConfig}
          disabled={loading}
          className="flex-1 px-4 py-2 rounded bg-gray-800 hover:bg-gray-700 text-white transition-colors disabled:opacity-50 text-sm"
        >
          {loading ? (
            <span className="flex items-center justify-center gap-2">
              <Loader2 className="w-4 h-4 animate-spin" />
              Validating...
            </span>
          ) : (
            'Validate Configuration'
          )}
        </button>
      </div>

      {/* Help Text */}
      <div className="mt-6 pt-4 border-t border-gray-700">
        <p className="text-xs text-gray-500">
          <strong>AP Mode:</strong> Creates a Wi-Fi hotspot. Connect to <span className="font-mono">Omega1-AP</span> and SSH to{' '}
          <span className="font-mono">omega1@192.168.4.1</span>
        </p>
        <p className="text-xs text-gray-500 mt-2">
          <strong>Client Mode:</strong> Connects to your existing Wi-Fi network. Use WiFi Networks section to scan and connect.
        </p>
      </div>

      {/* Connect Dialog */}
      {connectDialogOpen && selectedNetwork && (
        <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
          <div className="bg-gray-800 border border-gray-700 rounded-lg p-6 max-w-md w-full mx-4">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-lg font-semibold text-white">Connect to {selectedNetwork.ssid}</h3>
              <button
                onClick={() => {
                  setConnectDialogOpen(false);
                  setPassword('');
                  setSelectedNetwork(null);
                }}
                className="text-gray-400 hover:text-white"
              >
                <X className="w-5 h-5" />
              </button>
            </div>
            <div className="space-y-4">
              <div>
                <label className="text-sm text-gray-400 mb-1 block">Password</label>
                <input
                  type="password"
                  placeholder={selectedNetwork.security === 'Open' ? 'No password required' : 'Enter password'}
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  disabled={selectedNetwork.security === 'Open'}
                  className="w-full px-3 py-2 rounded bg-gray-700 border border-gray-600 text-white placeholder-gray-500 focus:outline-none focus:border-purple-500 disabled:opacity-50"
                />
                {selectedNetwork.security === 'Open' && (
                  <p className="text-xs text-gray-500 mt-1">This is an open network</p>
                )}
              </div>
              <div className="flex gap-2 justify-end">
                <button
                  onClick={() => {
                    setConnectDialogOpen(false);
                    setPassword('');
                    setSelectedNetwork(null);
                  }}
                  className="px-4 py-2 rounded bg-gray-700 hover:bg-gray-600 text-white transition-colors"
                >
                  Cancel
                </button>
                <button
                  onClick={() => {
                    if (selectedNetwork) {
                      connectToWiFi(selectedNetwork.ssid, password);
                    }
                  }}
                  disabled={connecting === selectedNetwork.ssid || (!password && selectedNetwork.security !== 'Open')}
                  className="px-4 py-2 rounded bg-purple-900/30 border border-purple-500/50 hover:bg-purple-900/50 text-white transition-colors disabled:opacity-50 disabled:cursor-not-allowed flex items-center gap-2"
                >
                  {connecting === selectedNetwork.ssid ? (
                    <>
                      <Loader2 className="w-4 h-4 animate-spin" />
                      Connecting...
                    </>
                  ) : (
                    'Connect'
                  )}
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
