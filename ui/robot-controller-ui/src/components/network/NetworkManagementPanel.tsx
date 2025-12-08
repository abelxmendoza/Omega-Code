/**
 * Comprehensive Network Management Panel
 * 
 * Provides full network control:
 * - WiFi scanning and connection
 * - AP/Client mode switching
 * - Network status display
 * - Tailscale status
 * - Hotspot quick connect
 * - AP fallback alerts
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
  Globe,
  Smartphone,
  Shield,
  AlertTriangle,
  X,
  Lock,
  Unlock,
} from 'lucide-react';
import { robotFetch } from '@/utils/network';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Input } from '@/components/ui/input';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from '@/components/ui/dialog';
import { Badge } from '@/components/ui/badge';

interface NetworkStatus {
  mode: 'ap' | 'client' | 'unknown';
  ssid?: string;
  ip?: string;
  signal?: number;
  interface?: string;
  ap_config?: {
    ssid?: string;
    ip?: string;
  };
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

export default function NetworkManagementPanel() {
  const [networkStatus, setNetworkStatus] = useState<NetworkStatus | null>(null);
  const [tailscaleStatus, setTailscaleStatus] = useState<TailscaleStatus | null>(null);
  const [wifiNetworks, setWifiNetworks] = useState<WiFiNetwork[]>([]);
  const [loading, setLoading] = useState(false);
  const [scanning, setScanning] = useState(false);
  const [switching, setSwitching] = useState(false);
  const [connecting, setConnecting] = useState<string | null>(null);
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

  // Fetch network status
  const fetchNetworkStatus = useCallback(async () => {
    try {
      const response = await robotFetch('/api/network');
      if (response.offline) {
        showToast('Robot backend unavailable', 'error');
        return;
      }
      const data = await response.json();
      if (data.ok !== false) {
        setNetworkStatus({
          mode: data.mode || 'unknown',
          ssid: data.ssid,
          ip: data.ip,
          signal: data.rssi,
          interface: data.interface,
          ap_config: data.ap_config,
        });
        
        // Check if AP mode is active (fallback scenario)
        if (data.mode === 'ap' && data.ap_config?.ssid === 'Omega1-AP') {
          setApFallbackActive(true);
        } else {
          setApFallbackActive(false);
        }
      }
    } catch (err) {
      console.error('Failed to fetch network status:', err);
    }
  }, [showToast]);

  // Fetch Tailscale status
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

  // Scan WiFi networks
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
        // Sort by signal strength
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

  // Switch network mode
  const switchMode = useCallback(async (mode: 'ap' | 'client') => {
    setSwitching(true);
    try {
      const response = await robotFetch('/api/network/mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
      });

      if (response.offline) {
        showToast('Robot backend unavailable', 'error');
        setSwitching(false);
        return;
      }

      const data = await response.json();
      if (data.ok) {
        showToast(`Switched to ${mode.toUpperCase()} mode`, 'success');
        setTimeout(() => {
          fetchNetworkStatus();
        }, 2000);
      } else {
        showToast(data.detail || 'Failed to switch mode', 'error');
      }
    } catch (err) {
      showToast('Error switching mode', 'error');
    } finally {
      setSwitching(false);
    }
  }, [fetchNetworkStatus, showToast]);

  // Connect to WiFi
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
          fetchNetworkStatus();
        }, 2000);
      } else {
        showToast(data.error || data.detail || 'Failed to connect', 'error');
      }
    } catch (err) {
      showToast('Error connecting to network', 'error');
    } finally {
      setConnecting(null);
    }
  }, [fetchNetworkStatus, showToast]);

  // Connect to hotspot
  const connectToHotspot = useCallback(async () => {
    if (!hotspotSSID.trim()) {
      showToast('Please enter hotspot SSID', 'error');
      return;
    }
    await connectToWiFi(hotspotSSID.trim(), hotspotPassword);
  }, [hotspotSSID, hotspotPassword, connectToWiFi, showToast]);

  // Restart Tailscale
  const restartTailscale = useCallback(async () => {
    try {
      // Use service restart endpoint if available
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

  // Initial load and periodic refresh
  useEffect(() => {
    fetchNetworkStatus();
    fetchTailscaleStatus();
    const interval = setInterval(() => {
      fetchNetworkStatus();
      fetchTailscaleStatus();
    }, 10000); // Refresh every 10 seconds
    return () => clearInterval(interval);
  }, [fetchNetworkStatus, fetchTailscaleStatus]);

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
    <div className="space-y-6">
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

      {/* AP Fallback Alert */}
      {apFallbackActive && (
        <Card className="border-yellow-500/50 bg-yellow-900/20">
          <CardContent className="pt-6">
            <div className="flex items-start gap-3">
              <AlertTriangle className="w-6 h-6 text-yellow-400 flex-shrink-0 mt-0.5" />
              <div className="flex-1">
                <h3 className="font-semibold text-yellow-200 mb-1">AP Fallback Mode Active</h3>
                <p className="text-sm text-yellow-300/80">
                  Robot booted into AP mode due to WiFi connection failure. Connect to{' '}
                  <span className="font-mono font-semibold">Omega1-AP</span> to configure WiFi or switch back to client mode.
                </p>
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Network Status Card */}
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <CardTitle className="flex items-center gap-2">
              <Wifi className="w-5 h-5" />
              Network Status
            </CardTitle>
            <Button
              variant="ghost"
              size="icon"
              onClick={() => {
                fetchNetworkStatus();
                fetchTailscaleStatus();
              }}
              disabled={loading}
            >
              <RefreshCw className={`w-4 h-4 ${loading ? 'animate-spin' : ''}`} />
            </Button>
          </div>
        </CardHeader>
        <CardContent className="space-y-4">
          {/* Mode */}
          <div className="flex items-center justify-between">
            <span className="text-sm text-muted-foreground">Mode</span>
            <Badge
              variant={networkStatus?.mode === 'ap' ? 'default' : 'secondary'}
              className={networkStatus?.mode === 'ap' ? 'bg-purple-600' : ''}
            >
              {networkStatus?.mode?.toUpperCase() || 'UNKNOWN'}
            </Badge>
          </div>

          {/* SSID */}
          {networkStatus?.ssid && (
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">SSID</span>
              <span className="font-mono text-sm">{networkStatus.ssid}</span>
            </div>
          )}

          {/* IP Address */}
          {networkStatus?.ip && (
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">IP Address</span>
              <span className="font-mono text-sm">{networkStatus.ip}</span>
            </div>
          )}

          {/* Signal Strength */}
          {networkStatus?.signal !== undefined && (
            <div className="flex items-center justify-between">
              <span className="text-sm text-muted-foreground">Signal Strength</span>
              <div className="flex items-center gap-2">
                {getSignalIcon(networkStatus.signal)}
                <span className="text-sm">{networkStatus.signal}%</span>
              </div>
            </div>
          )}

          {/* AP Config */}
          {networkStatus?.mode === 'ap' && networkStatus.ap_config && (
            <div className="pt-2 border-t">
              <div className="text-xs text-muted-foreground mb-2">Access Point Details</div>
              <div className="space-y-1 text-sm">
                <div className="flex justify-between">
                  <span className="text-muted-foreground">AP SSID:</span>
                  <span className="font-mono">{networkStatus.ap_config.ssid || 'Omega1-AP'}</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-muted-foreground">AP IP:</span>
                  <span className="font-mono">{networkStatus.ap_config.ip || '192.168.4.1'}</span>
                </div>
              </div>
            </div>
          )}
        </CardContent>
      </Card>

      {/* Mode Switcher */}
      <Card>
        <CardHeader>
          <CardTitle>Mode Switching</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 gap-3">
            <Button
              onClick={() => switchMode('ap')}
              disabled={switching || networkStatus?.mode === 'ap'}
              variant={networkStatus?.mode === 'ap' ? 'default' : 'outline'}
              className="h-auto py-4"
            >
              <div className="flex flex-col items-center gap-2">
                {switching && networkStatus?.mode !== 'ap' ? (
                  <Loader2 className="w-5 h-5 animate-spin" />
                ) : (
                  <Wifi className="w-5 h-5" />
                )}
                <span>Enable AP Mode</span>
                <span className="text-xs opacity-70">Create hotspot</span>
              </div>
            </Button>

            <Button
              onClick={() => switchMode('client')}
              disabled={switching || networkStatus?.mode === 'client'}
              variant={networkStatus?.mode === 'client' ? 'default' : 'outline'}
              className="h-auto py-4"
            >
              <div className="flex flex-col items-center gap-2">
                {switching && networkStatus?.mode !== 'client' ? (
                  <Loader2 className="w-5 h-5 animate-spin" />
                ) : (
                  <WifiOff className="w-5 h-5" />
                )}
                <span>Restore WiFi</span>
                <span className="text-xs opacity-70">Client mode</span>
              </div>
            </Button>
          </div>
        </CardContent>
      </Card>

      {/* WiFi Scanner */}
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <CardTitle>WiFi Networks</CardTitle>
            <Button onClick={scanWiFi} disabled={scanning} size="sm">
              {scanning ? (
                <>
                  <Loader2 className="w-4 h-4 mr-2 animate-spin" />
                  Scanning...
                </>
              ) : (
                <>
                  <RefreshCw className="w-4 h-4 mr-2" />
                  Scan Networks
                </>
              )}
            </Button>
          </div>
        </CardHeader>
        <CardContent>
          {wifiNetworks.length === 0 ? (
            <div className="text-center py-8 text-muted-foreground">
              <Wifi className="w-12 h-12 mx-auto mb-2 opacity-50" />
              <p>No networks found. Click &quot;Scan Networks&quot; to search.</p>
            </div>
          ) : (
            <div className="space-y-2 max-h-[400px] overflow-y-auto">
              {wifiNetworks.map((network) => (
                <div
                  key={network.ssid}
                  className="flex items-center justify-between p-3 rounded-lg border hover:bg-accent/50 cursor-pointer transition-colors"
                  onClick={() => {
                    setSelectedNetwork(network);
                    setConnectDialogOpen(true);
                  }}
                >
                  <div className="flex items-center gap-3 flex-1">
                    {getSecurityIcon(network.security)}
                    <div className="flex-1 min-w-0">
                      <div className="font-medium truncate">{network.ssid}</div>
                      <div className="text-xs text-muted-foreground">
                        {network.security || 'Open'} • {network.signal}%
                      </div>
                    </div>
                  </div>
                  <div className="flex items-center gap-2">
                    {getSignalIcon(network.signal)}
                    <Button
                      size="sm"
                      variant="outline"
                      onClick={(e) => {
                        e.stopPropagation();
                        setSelectedNetwork(network);
                        setConnectDialogOpen(true);
                      }}
                    >
                      Connect
                    </Button>
                  </div>
                </div>
              ))}
            </div>
          )}
        </CardContent>
      </Card>

      {/* Hotspot Quick Connect */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Smartphone className="w-5 h-5" />
            Hotspot Quick Connect
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-3">
          <div>
            <label className="text-sm font-medium mb-1 block">Hotspot SSID</label>
            <Input
              placeholder="iPhone's Name"
              value={hotspotSSID}
              onChange={(e) => setHotspotSSID(e.target.value)}
            />
          </div>
          <div>
            <label className="text-sm font-medium mb-1 block">Password</label>
            <Input
              type="password"
              placeholder="Hotspot password"
              value={hotspotPassword}
              onChange={(e) => setHotspotPassword(e.target.value)}
            />
          </div>
          <Button
            onClick={connectToHotspot}
            disabled={!hotspotSSID.trim() || connecting === hotspotSSID}
            className="w-full"
          >
            {connecting === hotspotSSID ? (
              <>
                <Loader2 className="w-4 h-4 mr-2 animate-spin" />
                Connecting...
              </>
            ) : (
              <>
                <Smartphone className="w-4 h-4 mr-2" />
                Connect to Hotspot
              </>
            )}
          </Button>
        </CardContent>
      </Card>

      {/* Tailscale Panel */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Shield className="w-5 h-5" />
            Tailscale VPN
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          {tailscaleStatus ? (
            <>
              <div className="flex items-center justify-between">
                <span className="text-sm text-muted-foreground">Status</span>
                <Badge
                  variant={tailscaleStatus.status === 'connected' ? 'default' : 'secondary'}
                  className={tailscaleStatus.status === 'connected' ? 'bg-green-600' : ''}
                >
                  {tailscaleStatus.status === 'connected' ? (
                    <>
                      <CheckCircle className="w-3 h-3 mr-1" />
                      Connected
                    </>
                  ) : (
                    <>
                      <XCircle className="w-3 h-3 mr-1" />
                      Disconnected
                    </>
                  )}
                </Badge>
              </div>

              {tailscaleStatus.ip && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-muted-foreground">Tailscale IP</span>
                  <span className="font-mono text-sm">{tailscaleStatus.ip}</span>
                </div>
              )}

              {tailscaleStatus.hostname && (
                <div className="flex items-center justify-between">
                  <span className="text-sm text-muted-foreground">Hostname</span>
                  <span className="font-mono text-sm">{tailscaleStatus.hostname}</span>
                </div>
              )}

              {!tailscaleStatus.enabled && (
                <div className="text-sm text-muted-foreground">
                  Tailscale is not installed or not enabled.
                </div>
              )}

              {tailscaleStatus.enabled && (
                <Button onClick={restartTailscale} variant="outline" className="w-full">
                  <RefreshCw className="w-4 h-4 mr-2" />
                  Restart Tailscale
                </Button>
              )}
            </>
          ) : (
            <div className="text-center py-4 text-muted-foreground">
              <Loader2 className="w-6 h-6 mx-auto mb-2 animate-spin" />
              <p>Loading Tailscale status...</p>
            </div>
          )}
        </CardContent>
      </Card>

      {/* Connect Dialog */}
      <Dialog open={connectDialogOpen} onOpenChange={setConnectDialogOpen}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Connect to {selectedNetwork?.ssid}</DialogTitle>
          </DialogHeader>
          <div className="space-y-4">
            <div>
              <label className="text-sm font-medium mb-1 block">Password</label>
              <Input
                type="password"
                placeholder={selectedNetwork?.security === 'Open' ? 'No password required' : 'Enter password'}
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                disabled={selectedNetwork?.security === 'Open'}
              />
              {selectedNetwork?.security === 'Open' && (
                <p className="text-xs text-muted-foreground mt-1">This is an open network</p>
              )}
            </div>
            <div className="flex gap-2 justify-end">
              <Button variant="outline" onClick={() => setConnectDialogOpen(false)}>
                Cancel
              </Button>
              <Button
                onClick={() => {
                  if (selectedNetwork) {
                    connectToWiFi(selectedNetwork.ssid, password);
                  }
                }}
                disabled={connecting === selectedNetwork?.ssid || (!password && selectedNetwork?.security !== 'Open')}
              >
                {connecting === selectedNetwork?.ssid ? (
                  <>
                    <Loader2 className="w-4 h-4 mr-2 animate-spin" />
                    Connecting...
                  </>
                ) : (
                  'Connect'
                )}
              </Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  );
}

