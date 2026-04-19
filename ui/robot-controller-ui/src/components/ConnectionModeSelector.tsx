'use client';

/**
 * ConnectionModeSelector — three-card mode picker shown on the dashboard.
 *
 * Modes:
 *   live          → connects to the physical Pi robot over the gateway
 *   sim-backend   → connects to a local sim server (localhost:8000) — requires
 *                   running the UI locally, not from a Vercel HTTPS deployment
 *   frontend-demo → fully client-side SimEngine, no server needed
 */

import React, { useEffect, useState } from 'react';
import { Radio, Server, FlaskConical, AlertTriangle, CheckCircle2 } from 'lucide-react';
import { useDemoMode, ConnectionMode } from '@/context/DemoModeContext';

interface ModeCardProps {
  id:          ConnectionMode;
  icon:        React.ReactNode;
  title:       string;
  description: string;
  badge?:      React.ReactNode;
  active:      boolean;
  onClick:     () => void;
}

function ModeCard({ id, icon, title, description, badge, active, onClick }: ModeCardProps) {
  return (
    <button
      key={id}
      onClick={onClick}
      className={[
        'relative flex flex-col gap-2 rounded-xl border p-4 text-left transition-all w-full',
        active
          ? 'border-violet-500/70 bg-violet-900/25 ring-1 ring-violet-500/30'
          : 'border-gray-700/60 bg-gray-800/40 hover:border-gray-600 hover:bg-gray-800/70',
      ].join(' ')}
    >
      <div className="flex items-center justify-between">
        <div className={`flex items-center gap-2 font-semibold text-sm ${active ? 'text-violet-200' : 'text-gray-300'}`}>
          <span className={active ? 'text-violet-400' : 'text-gray-400'}>{icon}</span>
          {title}
        </div>
        {active && (
          <CheckCircle2 size={14} className="text-violet-400 flex-shrink-0" />
        )}
      </div>
      <p className="text-xs text-gray-400 leading-relaxed">{description}</p>
      {badge}
    </button>
  );
}

interface ConnectionModeSelectorProps {
  className?: string;
}

const ConnectionModeSelector: React.FC<ConnectionModeSelectorProps> = ({ className = '' }) => {
  const { connectionMode, setConnectionMode, isHydrated } = useDemoMode();
  const [isHttps, setIsHttps] = useState(false);

  useEffect(() => {
    setIsHttps(typeof window !== 'undefined' && window.location.protocol === 'https:');
  }, []);

  if (!isHydrated) return null;

  const simBackendWarning = isHttps && (
    <div className="flex items-start gap-1.5 rounded-lg border border-amber-600/40 bg-amber-900/20 px-2 py-1.5 text-[11px] text-amber-300 leading-tight">
      <AlertTriangle size={11} className="mt-0.5 flex-shrink-0" />
      <span>
        Requires running the UI locally (<code className="font-mono">npm run dev</code>).
        Browsers block HTTP localhost from HTTPS pages.
      </span>
    </div>
  );

  return (
    <div className={['flex flex-col gap-3', className].join(' ')}>
      <p className="text-[11px] text-gray-500 uppercase tracking-wider font-semibold">Connection Mode</p>

      <div className="grid grid-cols-1 gap-2 sm:grid-cols-3">
        <ModeCard
          id="live"
          icon={<Radio size={14} />}
          title="Live Robot"
          description="Connects to the physical Omega-1 robot over the network gateway. Requires Wi-Fi access to the Pi."
          active={connectionMode === 'live'}
          onClick={() => setConnectionMode('live')}
        />

        <ModeCard
          id="sim-backend"
          icon={<Server size={14} />}
          title="Sim Backend"
          description="Connects to a local simulation server (localhost:8000). Full backend physics, no physical robot needed."
          badge={simBackendWarning}
          active={connectionMode === 'sim-backend'}
          onClick={() => setConnectionMode('sim-backend')}
        />

        <ModeCard
          id="frontend-demo"
          icon={<FlaskConical size={14} />}
          title="Browser Demo"
          description="Fully client-side demo — no server or robot needed. Works from any URL including the Vercel deployment."
          active={connectionMode === 'frontend-demo'}
          onClick={() => setConnectionMode('frontend-demo')}
        />
      </div>

      {connectionMode === 'sim-backend' && (
        <div className="rounded-lg border border-blue-600/30 bg-blue-900/15 px-3 py-2 text-xs text-blue-300">
          <strong className="font-semibold">Sim Backend active.</strong> Start the server locally:
          <code className="block mt-1 font-mono text-[11px] text-blue-200 bg-blue-950/50 rounded px-2 py-1">
            cd servers/robot_controller_backend && ./scripts/start_sim_local.sh
          </code>
        </div>
      )}
    </div>
  );
};

export default ConnectionModeSelector;
