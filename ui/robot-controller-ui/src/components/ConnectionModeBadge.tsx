'use client';

/**
 * ConnectionModeBadge — compact header pill showing the active connection mode.
 * Clicking it opens a dropdown with the three-card ConnectionModeSelector.
 * Closes on outside click or after selecting a mode.
 */

import React, { useEffect, useRef, useState } from 'react';
import { ChevronDown, FlaskConical, Radio, Server } from 'lucide-react';
import { useDemoMode, ConnectionMode } from '@/context/DemoModeContext';
import ConnectionModeSelector from './ConnectionModeSelector';

const MODE_META: Record<ConnectionMode, { label: string; icon: React.ReactNode; colors: string }> = {
  live: {
    label:  'LIVE',
    icon:   <Radio size={11} className="flex-shrink-0" />,
    colors: 'bg-white/10 border-white/15 text-white/90 hover:bg-white/15',
  },
  'sim-backend': {
    label:  'SIM',
    icon:   <Server size={11} className="flex-shrink-0" />,
    colors: 'bg-blue-900/50 border-blue-500/50 text-blue-200 hover:bg-blue-900/70',
  },
  'frontend-demo': {
    label:  'DEMO',
    icon:   <FlaskConical size={11} className="flex-shrink-0" />,
    colors: 'bg-violet-900/60 border-violet-500/60 text-violet-300 hover:bg-violet-900/80',
  },
};

const ConnectionModeBadge: React.FC = () => {
  const { connectionMode, isHydrated } = useDemoMode();
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);

  // Close on outside click
  useEffect(() => {
    if (!open) return;
    const handler = (e: MouseEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) {
        setOpen(false);
      }
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [open]);

  // Close on Escape
  useEffect(() => {
    if (!open) return;
    const handler = (e: KeyboardEvent) => { if (e.key === 'Escape') setOpen(false); };
    document.addEventListener('keydown', handler);
    return () => document.removeEventListener('keydown', handler);
  }, [open]);

  if (!isHydrated) return null;

  const meta = MODE_META[connectionMode];

  return (
    <div ref={ref} className="relative">
      <button
        onClick={() => setOpen((v) => !v)}
        title="Switch connection mode"
        className={[
          'flex items-center gap-1.5 text-[10px] xl4:text-sm px-1.5 xl4:px-2 py-0.5 xl4:py-1',
          'rounded border font-semibold transition-all select-none',
          meta.colors,
        ].join(' ')}
      >
        {meta.icon}
        <span>{meta.label}</span>
        {connectionMode === 'frontend-demo' && (
          <span className="w-1.5 h-1.5 rounded-full bg-violet-400 animate-pulse flex-shrink-0" />
        )}
        <ChevronDown
          size={10}
          className={`flex-shrink-0 transition-transform ${open ? 'rotate-180' : ''}`}
        />
      </button>

      {open && (
        <div className="absolute left-0 top-full mt-2 z-50 w-[600px] max-w-[90vw]
                        rounded-xl border border-gray-700 bg-gray-900 shadow-2xl p-4"
        >
          <ConnectionModeSelector onSelect={() => setOpen(false)} />
        </div>
      )}
    </div>
  );
};

export default ConnectionModeBadge;
