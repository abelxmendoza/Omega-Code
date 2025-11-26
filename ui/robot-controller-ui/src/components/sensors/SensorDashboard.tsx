// File: src/components/sensors/SensorDashboard.tsx

/**
 * SensorDashboard Component
 *
 * Displays real-time sensor data received over WebSocket:
 * - Line Tracking Sensor (IR01, IR02, IR03)
 * - Ultrasonic Sensor (distance in cm, m, inches, feet)
 *
 * Tries Tailscale first, falls back to LAN. Works with both
 * separate or combined servers for line tracking & ultrasonic.
 */

import React, { useEffect, useState, useRef } from 'react';
import {
  connectLineTrackerWs,
  startJsonHeartbeat,
  parseLineTrackingPayload,
} from '@/utils/connectLineTrackerWs';
import { resolveWsUrl, resolveWsCandidates } from '@/utils/resolveWsUrl';
import { Button } from '@/components/ui/button';
import { Radar } from 'lucide-react';
import UltrasonicVisualization from './UltrasonicVisualization';

interface LineTrackingData {
  IR01: number;
  IR02: number;
  IR03: number;
}
interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

type ServerStatus = 'connecting' | 'connected' | 'disconnected';

// ------------- WebSocket URLs (from env) ----------------
// Use resolveWsUrl with default port 8080 and path /ultrasonic for direct Go server connection
const ULTRASONIC_WS = resolveWsUrl('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
  defaultPort: '8080',
  path: '/ultrasonic',
});

// Small status dot helper
function StatusDot({
  status,
  title,
}: {
  status: ServerStatus;
  title: string;
}) {
  const color =
    status === 'connected'
      ? 'bg-emerald-500'
      : status === 'connecting'
      ? 'bg-slate-500'
      : 'bg-rose-500';
  return (
    <span
      className={`inline-block rounded-full ${color} shrink-0`}
      style={{ width: 8, height: 8 }} // smaller than w-2/h-2 (8px)
      title={title}
      aria-label={title}
    />
  );
}

const SensorDashboard: React.FC = () => {
  // --- State ---
  const [lineTrackingData, setLineTrackingData] = useState<LineTrackingData>({
    IR01: 0,
    IR02: 0,
    IR03: 0,
  });
  const [ultrasonicData, setUltrasonicData] = useState<UltrasonicData>({
    distance_cm: 0,
    distance_m: 0,
    distance_inch: 0,
    distance_feet: 0,
  });

  // Connection statuses
  const [lineStatus, setLineStatus] = useState<ServerStatus>('disconnected');
  const [lineLatencyMs, setLineLatencyMs] = useState<number | null>(null);
  const [ultraStatus, setUltraStatus] = useState<ServerStatus>('disconnected');
  
  // Ultrasonic error tracking
  const [ultraError, setUltraError] = useState<string | null>(null);
  const [ultraErrorCount, setUltraErrorCount] = useState(0);
  const [ultraLastSuccess, setUltraLastSuccess] = useState<Date | null>(null);
  const [ultraConsecutiveErrors, setUltraConsecutiveErrors] = useState(0);
  
  // Ultrasonic visualization popup state
  const [showUltraVisualization, setShowUltraVisualization] = useState(false);

  // --- WebSocket refs ---
  const lineWs = useRef<WebSocket | null>(null);
  const stopLineHeartbeat = useRef<null | (() => void)>(null);
  const ultrasonicWs = useRef<WebSocket | null>(null);

  // Line tracker WS (via helper)
  useEffect(() => {
    let cancelled = false;

    (async () => {
      try {
        setLineStatus('connecting');
        const ws = await connectLineTrackerWs();
        if (cancelled) {
          try { ws.close(); } catch {}
          return;
        }
        lineWs.current = ws;
        setLineStatus('connected');
        setLineLatencyMs(null);

        // Start JSON ping/pong heartbeat
        stopLineHeartbeat.current = startJsonHeartbeat(ws, {
          onLatency: (ms) => setLineLatencyMs(ms),
          onDisconnect: () => setLineStatus('disconnected'),
        });

        ws.addEventListener('message', (event) => {
          try {
            const data = JSON.parse(event.data);
            if (data?.status === 'connected' && data?.service === 'line-tracker') {
              setLineStatus('connected');
              return;
            }
            const normalized = parseLineTrackingPayload(data);
            if (normalized) setLineTrackingData(normalized);
          } catch {
            /* ignore */
          }
        });

        ws.onclose = () => setLineStatus('disconnected');
        ws.onerror = () => setLineStatus('disconnected');
      } catch (e) {
        console.warn('[LINE] connect failed:', e);
        setLineStatus('disconnected');
      }
    })();

    return () => {
      cancelled = true;
      try { stopLineHeartbeat.current?.(); } catch {}
      try { lineWs.current?.close(); } catch {}
      stopLineHeartbeat.current = null;
      lineWs.current = null;
    };
  }, []);

  // Ultrasonic WS (direct connection to Go server on port 8080)
  useEffect(() => {
    // Build candidates: direct env vars first, then fallback to host-based URLs
    const candidates = resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', {
      defaultPort: '8080',
      path: '/ultrasonic',
    });
    
    if (!candidates.length) {
      const errorMsg = 'No WebSocket URL candidates found. Check NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_* in .env.local';
      console.error(`[ULTRASONIC] ❌ ${errorMsg}`);
      setUltraError(errorMsg);
      setUltraStatus('disconnected');
      return;
    }
    
    console.log(`[ULTRASONIC] 🔍 Found ${candidates.length} connection candidate(s):`, candidates);

    let currentCandidateIndex = 0;
    let ws: WebSocket | null = null;
    let reconnectTimeout: NodeJS.Timeout | null = null;

    const tryConnect = () => {
      if (currentCandidateIndex >= candidates.length) {
        console.error('[ULTRASONIC] All connection attempts failed');
        setUltraStatus('disconnected');
        return;
      }

      const url = candidates[currentCandidateIndex];
      console.log(`[ULTRASONIC] Attempting connection ${currentCandidateIndex + 1}/${candidates.length}: ${url}`);
      setUltraStatus('connecting');

      try {
        ws = new WebSocket(url);
        ultrasonicWs.current = ws;

        ws.onopen = () => {
          setUltraStatus('connected');
          console.log(`[ULTRASONIC] ✅ WebSocket connected: ${url}`);
          currentCandidateIndex = 0; // Reset on success
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            // Ignore welcome message
            if (data.status === 'connected' && data.service === 'ultrasonic') {
              console.log('[ULTRASONIC] ✅ Welcome message received - sensor ready');
              setUltraError(null);
              return;
            }
            
            // Handle error messages
            if (data.status === 'error' || data.error) {
              const errorMsg = data.error || 'Unknown sensor error';
              setUltraError(errorMsg);
              setUltraErrorCount(prev => {
                const newCount = prev + 1;
                console.warn(`[ULTRASONIC] ⚠️  Sensor error: ${errorMsg}`);
                console.warn(`[ULTRASONIC] Total errors: ${newCount}`);
                return newCount;
              });
              setUltraConsecutiveErrors(prev => {
                const newConsecutive = prev + 1;
                console.warn(`[ULTRASONIC] Consecutive errors: ${newConsecutive}`);
                
                // Show warning after multiple consecutive errors
                if (newConsecutive >= 4) {
                  console.error(`[ULTRASONIC] 🔧 Hardware issue detected! Check: 1) Power (5V), 2) Wiring (GPIO27/22), 3) Run diagnostic: go run test_ultrasonic_hardware.go`);
                }
                return newConsecutive;
              });
              return;
            }
            
            // Handle successful readings
            if (data.distance_cm !== undefined && data.status !== 'error') {
              const newData = {
                distance_cm: Number(data.distance_cm ?? 0),
                distance_m: Number(data.distance_m ?? 0),
                distance_inch: Number(data.distance_inch ?? 0),
                distance_feet: Number(data.distance_feet ?? 0),
              };
              
              // Validate reading
              if (newData.distance_cm < 0 || newData.distance_cm > 400) {
                console.warn(`[ULTRASONIC] ⚠️  Invalid reading: ${newData.distance_cm} cm (out of range)`);
                setUltraError(`Invalid reading: ${newData.distance_cm} cm (valid range: 2-400cm)`);
                return;
              }
              
              setUltrasonicData(newData);
              setUltraError(null);
              setUltraLastSuccess(new Date());
              
              setUltraConsecutiveErrors(prev => {
                if (prev > 0) {
                  console.log(`[ULTRASONIC] ✅ Sensor recovered after ${prev} errors`);
                }
                return 0;
              });
              
              console.log(`[ULTRASONIC] 📏 Distance: ${newData.distance_cm} cm (${newData.distance_m.toFixed(2)}m)`);
            }
          } catch (err) {
            console.error('[ULTRASONIC] ❌ Failed to parse message:', err, 'Raw:', event.data);
            setUltraError(`Parse error: ${err instanceof Error ? err.message : 'Unknown'}`);
          }
        };

        ws.onerror = (e) => {
          console.error(`[ULTRASONIC] ❌ WebSocket error on ${url}:`, e);
          setUltraError(`Connection error: ${url}`);
          setUltraStatus('disconnected');
        };

        ws.onclose = (event) => {
          const wasClean = event.wasClean;
          const code = event.code;
          const reason = event.reason || 'No reason provided';
          
          console.log(`[ULTRASONIC] WebSocket closed: ${url} (clean: ${wasClean}, code: ${code}, reason: ${reason})`);
          setUltraStatus('disconnected');
          
          if (!wasClean && code !== 1000) {
            console.warn(`[ULTRASONIC] Unexpected close: code ${code}, reason: ${reason}`);
            setUltraError(`Connection closed unexpectedly (code: ${code})`);
          }
          
          // Try next candidate after a delay
          if (currentCandidateIndex < candidates.length - 1) {
            console.log(`[ULTRASONIC] Retrying with next candidate in 2s...`);
            currentCandidateIndex++;
            reconnectTimeout = setTimeout(tryConnect, 2000);
          } else {
            console.error(`[ULTRASONIC] ❌ All connection attempts exhausted. Check:`);
            console.error(`[ULTRASONIC]   1. Server running: go run main_ultrasonic.go`);
            console.error(`[ULTRASONIC]   2. Port 8080 accessible`);
            console.error(`[ULTRASONIC]   3. Firewall allows connections`);
            setUltraError(`All connection attempts failed. Check server is running on port 8080.`);
          }
        };
      } catch (err) {
        const errorMsg = err instanceof Error ? err.message : String(err);
        console.error(`[ULTRASONIC] ❌ Failed to create WebSocket for ${url}:`, errorMsg);
        setUltraError(`Failed to create connection: ${errorMsg}`);
        setUltraStatus('disconnected');
        
        if (currentCandidateIndex < candidates.length - 1) {
          currentCandidateIndex++;
          reconnectTimeout = setTimeout(tryConnect, 2000);
        } else {
          console.error(`[ULTRASONIC] ❌ All candidates failed. Last error: ${errorMsg}`);
          setUltraError(`All connection attempts failed: ${errorMsg}`);
        }
      }
    };

    tryConnect();

    return () => {
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      if (ws) ws.close();
      ultrasonicWs.current = null;
    };
  }, []);

  // Tooltip strings
  const lineTitle =
    lineStatus === 'connected'
      ? lineLatencyMs != null
        ? `Line tracker: Connected • ${lineLatencyMs}ms`
        : 'Line tracker: Connected'
      : `Line tracker: ${lineStatus[0].toUpperCase()}${lineStatus.slice(1)}`;

  const ultraTitle = ultraError 
    ? `Ultrasonic: ${ultraStatus} • Error: ${ultraError.substring(0, 50)}...`
    : `Ultrasonic: ${ultraStatus}${ultraLastSuccess ? ` • Last success: ${ultraLastSuccess.toLocaleTimeString()}` : ''}${ultraErrorCount > 0 ? ` • Errors: ${ultraErrorCount}` : ''}`;

  return (
    <div className="sensor-dashboard bg-gray-800 text-white p-4 rounded-lg shadow-md max-w-6xl mx-auto">
      <h2 className="text-lg font-bold mb-4 text-center">Sensor Dashboard</h2>

      <div className="sensor-container grid grid-cols-1 sm:grid-cols-2 gap-6">
        {/* Line Tracking */}
        <div className="line-tracking bg-gray-900 p-4 rounded-lg shadow-lg">
          <div className="flex items-center justify-between mb-2">
            <strong className="block text-sm underline">Line Tracking Sensor</strong>
            <StatusDot status={lineStatus} title={lineTitle} />
          </div>
          <p>IR01: {lineTrackingData.IR01}</p>
          <p>IR02: {lineTrackingData.IR02}</p>
          <p>IR03: {lineTrackingData.IR03}</p>
        </div>

        {/* Ultrasonic */}
        <div className="ultrasonic-distance bg-gray-900 p-4 rounded-lg shadow-lg">
          <div className="flex items-center justify-between mb-2">
            <strong className="block text-sm underline">Ultrasonic Distance</strong>
            <StatusDot 
              status={ultraError && ultraConsecutiveErrors >= 3 ? 'disconnected' : ultraStatus} 
              title={ultraTitle} 
            />
          </div>
          
          {/* Error Display - Logged to console instead */}
          {ultraError && (
            (() => {
              console.error('SensorDashboard Ultrasonic Error:', ultraError, { 
                consecutiveErrors: ultraConsecutiveErrors,
                troubleshooting: ultraConsecutiveErrors >= 3 ? [
                  'Check sensor power: VCC → 5V, GND → GND',
                  'Verify wiring: Trigger → GPIO27 (Pin13), Echo → GPIO22 (Pin15)',
                  'Run diagnostic: go run test_ultrasonic_hardware.go',
                  'Check GPIO permissions: sudo usermod -a -G gpio $USER'
                ] : []
              });
              return null;
            })()
          )}
          
          {/* Success/Status Indicator */}
          {!ultraError && ultraStatus === 'connected' && (
            <div className="mb-2 text-xs text-green-400">
              ✅ Sensor active{ultraLastSuccess ? ` • Last reading: ${ultraLastSuccess.toLocaleTimeString()}` : ''}
            </div>
          )}
          
          {/* Distance Display */}
          {ultraError ? (
            <div className="text-gray-500 italic">
              <p>No valid reading</p>
              <p className="text-xs mt-1">Check error message above</p>
            </div>
          ) : (
            <>
              <p className={ultrasonicData.distance_cm === 0 ? 'text-yellow-400' : 'text-white'}>
                {ultrasonicData.distance_cm} cm
                {ultrasonicData.distance_cm === 0 && ' (may be invalid)'}
              </p>
              <p>{ultrasonicData.distance_m.toFixed(2)} m</p>
              <p>{ultrasonicData.distance_inch.toFixed(2)} inches</p>
              <p>{ultrasonicData.distance_feet.toFixed(2)} feet</p>
            </>
          )}
          
          {/* Error Statistics */}
          {ultraErrorCount > 0 && (
            <div className="mt-2 text-xs text-gray-400">
              Errors: {ultraErrorCount} | Consecutive: {ultraConsecutiveErrors}
            </div>
          )}
          
          {/* Ultrasonic Visualization Popup */}
          <div className="relative mt-3">
            <Button
              className="w-full gap-2 bg-blue-600 hover:bg-blue-700 text-white"
              variant="default"
              onClick={(e) => {
                e.stopPropagation();
                setShowUltraVisualization(!showUltraVisualization);
              }}
            >
              <Radar className="h-4 w-4" />
              {showUltraVisualization ? 'Hide Visualization' : 'Show Visualization'}
            </Button>
            {/* Popup - shows when clicked */}
            {showUltraVisualization && (
              <>
                {/* Backdrop */}
                <div 
                  className="fixed inset-0 bg-black/50 z-[99]"
                  onClick={() => setShowUltraVisualization(false)}
                />
                {/* Popup centered */}
                <div 
                  className="fixed top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-[500px] max-w-[90vw] max-h-[90vh] overflow-auto transition-all duration-200 z-[100]"
                  onClick={(e) => e.stopPropagation()}
                >
                  <div className="bg-gray-900 border-2 border-green-500 rounded-lg shadow-xl p-3 relative">
                    <button
                      onClick={() => setShowUltraVisualization(false)}
                      className="absolute top-2 right-2 text-gray-400 hover:text-white z-10 text-2xl leading-none w-6 h-6 flex items-center justify-center rounded hover:bg-gray-700"
                      aria-label="Close"
                    >
                      ×
                    </button>
                    <UltrasonicVisualization
                      data={ultrasonicData}
                      isConnected={ultraStatus === 'connected'}
                    />
                  </div>
                </div>
              </>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default SensorDashboard;
