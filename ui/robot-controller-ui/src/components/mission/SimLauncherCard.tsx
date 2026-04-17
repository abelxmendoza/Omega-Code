'use client';

/**
 * SimLauncherCard — shows local backend status and lets you start/stop it.
 *
 * Polls /api/sim-launcher every 4 seconds.  Intended for the Mission page
 * sidebar so you can run the sim without the robot present.
 */

import React, { useCallback, useEffect, useRef, useState } from 'react';

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

type BackendStatus =
  | 'checking'
  | 'offline'
  | 'running_no_sim'
  | 'running_sim';

interface LauncherState {
  status:        BackendStatus;
  ownedProcess:  boolean;
  pid:           number | null;
  actionPending: boolean;
  error:         string | null;
}

// ---------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------

export default function SimLauncherCard() {
  const [state, setState] = useState<LauncherState>({
    status:        'checking',
    ownedProcess:  false,
    pid:           null,
    actionPending: false,
    error:         null,
  });

  const pollRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  // ── poll status ──────────────────────────────────────────────────────────
  const poll = useCallback(async () => {
    try {
      const res  = await fetch('/api/sim-launcher');
      const data = await res.json() as {
        alive: boolean; simMode: boolean; pid: number | null; ownedProcess: boolean;
      };
      setState(s => ({
        ...s,
        status:       !data.alive ? 'offline'
                    : data.simMode ? 'running_sim'
                    : 'running_no_sim',
        ownedProcess: data.ownedProcess,
        pid:          data.pid,
        error:        null,
      }));
    } catch {
      setState(s => ({ ...s, status: 'offline' }));
    }
  }, []);

  useEffect(() => {
    poll();
    const scheduleNext = () => { pollRef.current = setTimeout(async () => { await poll(); scheduleNext(); }, 4000); };
    scheduleNext();
    return () => { if (pollRef.current) clearTimeout(pollRef.current); };
  }, [poll]);

  // ── actions ──────────────────────────────────────────────────────────────
  const startSim = useCallback(async () => {
    setState(s => ({ ...s, actionPending: true, error: null }));
    try {
      const res  = await fetch('/api/sim-launcher', { method: 'POST' });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? `HTTP ${res.status}`);
      setState(s => ({ ...s, status: 'checking' }));
      // poll immediately after a short boot delay
      setTimeout(poll, 2500);
    } catch (e) {
      setState(s => ({ ...s, error: e instanceof Error ? e.message : String(e) }));
    } finally {
      setState(s => ({ ...s, actionPending: false }));
    }
  }, [poll]);

  const stopSim = useCallback(async () => {
    setState(s => ({ ...s, actionPending: true, error: null }));
    try {
      const res  = await fetch('/api/sim-launcher', { method: 'DELETE' });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error ?? `HTTP ${res.status}`);
      setState(s => ({ ...s, status: 'checking' }));
      setTimeout(poll, 1500);
    } catch (e) {
      setState(s => ({ ...s, error: e instanceof Error ? e.message : String(e) }));
    } finally {
      setState(s => ({ ...s, actionPending: false }));
    }
  }, [poll]);

  // ── derived display ──────────────────────────────────────────────────────
  const { status, ownedProcess, pid, actionPending, error } = state;

  const badge = {
    checking:       { label: 'Checking…',        cls: 'bg-gray-700 text-gray-400',                  dot: 'bg-gray-500 animate-pulse' },
    offline:        { label: 'Backend Offline',   cls: 'bg-red-900/40 border border-red-700/40 text-red-400',   dot: 'bg-red-500' },
    running_no_sim: { label: 'Running — No Sim',  cls: 'bg-amber-900/40 border border-amber-700/40 text-amber-400', dot: 'bg-amber-400' },
    running_sim:    { label: 'Sim Active',         cls: 'bg-emerald-900/40 border border-emerald-700/40 text-emerald-400', dot: 'bg-emerald-400' },
  }[status];

  return (
    <div className="bg-gray-800 rounded-lg p-3 border border-gray-700">
      <div className="flex items-center justify-between mb-2">
        <span className="text-xs font-semibold uppercase tracking-wide text-gray-400">
          Sim Backend
        </span>
        {pid !== null && (
          <span className="text-[10px] text-gray-600 font-mono">pid {pid}</span>
        )}
      </div>

      {/* Status badge */}
      <div className={`flex items-center gap-1.5 px-2 py-1 rounded text-xs mb-2.5 ${badge.cls}`}>
        <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${badge.dot}`} />
        <span>{badge.label}</span>
      </div>

      {/* Action buttons */}
      <div className="flex gap-2">
        {status === 'offline' && (
          <button
            onClick={startSim}
            disabled={actionPending}
            className="flex-1 py-1.5 rounded text-xs font-semibold bg-emerald-700 hover:bg-emerald-600 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
          >
            {actionPending ? 'Starting…' : 'Start Sim Backend'}
          </button>
        )}

        {status === 'running_no_sim' && !ownedProcess && (
          <p className="text-[11px] text-amber-400/80">
            Backend running without SIM_MODE — restart it with <code className="text-amber-300">SIM_MODE=1</code>.
          </p>
        )}

        {(status === 'running_sim' || status === 'running_no_sim') && ownedProcess && (
          <button
            onClick={stopSim}
            disabled={actionPending}
            className="flex-1 py-1.5 rounded text-xs font-semibold bg-gray-700 hover:bg-red-700/80 disabled:opacity-50 disabled:cursor-not-allowed transition-colors text-gray-300 hover:text-white"
          >
            {actionPending ? 'Stopping…' : 'Stop Backend'}
          </button>
        )}
      </div>

      {error && (
        <p className="mt-2 text-[11px] text-red-400 break-words">{error}</p>
      )}
    </div>
  );
}
