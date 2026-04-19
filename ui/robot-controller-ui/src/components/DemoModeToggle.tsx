'use client';

/**
 * DemoModeToggle — pill button that switches between DEMO and LIVE modes.
 *
 * In DEMO mode the SimEngine runs and all hardware connections are suppressed.
 * In LIVE mode the normal WebSocket / HTTP stack is active.
 */

import React from 'react';
import { FlaskConical, Radio } from 'lucide-react';
import { useDemoMode } from '@/context/DemoModeContext';

interface DemoModeToggleProps {
  className?: string;
}

const DemoModeToggle: React.FC<DemoModeToggleProps> = ({ className = '' }) => {
  const { demoMode, setDemoMode } = useDemoMode();

  return (
    <button
      onClick={() => setDemoMode(!demoMode)}
      title={demoMode ? 'Demo Mode active — click to go live' : 'Click to enable Demo Mode (no hardware needed)'}
      className={[
        'flex items-center gap-1.5 px-2.5 py-1 rounded-full border text-xs font-semibold transition-all select-none',
        demoMode
          ? 'bg-violet-900/60 border-violet-500/60 text-violet-300 hover:bg-violet-900/80'
          : 'bg-gray-800/60 border-gray-600/50 text-gray-400 hover:border-gray-500 hover:text-gray-200',
        className,
      ].join(' ')}
    >
      {demoMode ? (
        <>
          <FlaskConical size={11} className="flex-shrink-0" />
          <span>DEMO</span>
          <span className="w-1.5 h-1.5 rounded-full bg-violet-400 animate-pulse flex-shrink-0" />
        </>
      ) : (
        <>
          <Radio size={11} className="flex-shrink-0" />
          <span>LIVE</span>
        </>
      )}
    </button>
  );
};

export default DemoModeToggle;
