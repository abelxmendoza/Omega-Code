/*
 * XboxControllerStatus.tsx
 * ========================
 * Debug panel for the Xbox controller connected to the operator laptop.
 * Shows:
 *   - Connection status + controller name
 *   - Left/right analogue sticks (dot-in-circle)
 *   - LT / RT trigger bars
 *   - A B X Y  LB RB  D-Pad button states
 *   - Computed drive vector (linear / angular) with bar visualization
 *   - E-Stop indicator when A is held
 */

'use client';

import React from 'react';
import { GamepadState } from '../../hooks/useGamepad';

interface Props {
  state: GamepadState;
  paused: boolean;
  onTogglePause: () => void;
}

// --- sub-components ---------------------------------------------------------

const StickViz: React.FC<{ x: number; y: number; label: string }> = ({ x, y, label }) => {
  const cx = 32 + x * 26;
  const cy = 32 + y * 26;
  return (
    <div className="flex flex-col items-center gap-1">
      <svg width="64" height="64" className="block">
        {/* outer ring */}
        <circle cx="32" cy="32" r="28" fill="none" stroke="rgba(255,255,255,0.12)" strokeWidth="1.5" />
        {/* center cross */}
        <line x1="32" y1="8" x2="32" y2="56" stroke="rgba(255,255,255,0.06)" strokeWidth="1" />
        <line x1="8" y1="32" x2="56" y2="32" stroke="rgba(255,255,255,0.06)" strokeWidth="1" />
        {/* dot */}
        <circle
          cx={cx}
          cy={cy}
          r="6"
          fill={Math.abs(x) > 0.02 || Math.abs(y) > 0.02 ? '#C400FF' : 'rgba(196,0,255,0.35)'}
        />
      </svg>
      <span className="text-[10px] text-white/50">{label}</span>
      <span className="text-[9px] font-mono text-white/40">
        {x.toFixed(2)}, {y.toFixed(2)}
      </span>
    </div>
  );
};

const TriggerBar: React.FC<{ value: number; label: string; color: string }> = ({ value, label, color }) => (
  <div className="flex flex-col items-center gap-1 w-12">
    <span className="text-[10px] text-white/50">{label}</span>
    <div className="w-4 h-20 bg-white/5 rounded-full overflow-hidden flex flex-col-reverse border border-white/10">
      <div
        className="w-full rounded-full transition-none"
        style={{ height: `${value * 100}%`, backgroundColor: color }}
      />
    </div>
    <span className="text-[9px] font-mono text-white/40">{value.toFixed(2)}</span>
  </div>
);

const Btn: React.FC<{ active: boolean; label: string; color?: string }> = ({ active, label, color = '#C400FF' }) => (
  <div
    className="w-8 h-8 rounded-full flex items-center justify-center text-[10px] font-bold border transition-none"
    style={{
      backgroundColor: active ? color : 'rgba(255,255,255,0.05)',
      borderColor:     active ? color : 'rgba(255,255,255,0.12)',
      color:           active ? '#fff' : 'rgba(255,255,255,0.35)',
      boxShadow:       active ? `0 0 8px ${color}88` : 'none',
    }}
  >
    {label}
  </div>
);

const DPad: React.FC<{ up: boolean; down: boolean; left: boolean; right: boolean }> = ({ up, down, left, right }) => (
  <div className="grid grid-cols-3 gap-0.5 w-fit">
    <div />
    <div className={`w-5 h-5 rounded-sm flex items-center justify-center text-[8px] ${up ? 'bg-white/40' : 'bg-white/5 border border-white/10'}`}>▲</div>
    <div />
    <div className={`w-5 h-5 rounded-sm flex items-center justify-center text-[8px] ${left ? 'bg-white/40' : 'bg-white/5 border border-white/10'}`}>◀</div>
    <div className="w-5 h-5 rounded-full bg-white/5 border border-white/10" />
    <div className={`w-5 h-5 rounded-sm flex items-center justify-center text-[8px] ${right ? 'bg-white/40' : 'bg-white/5 border border-white/10'}`}>▶</div>
    <div />
    <div className={`w-5 h-5 rounded-sm flex items-center justify-center text-[8px] ${down ? 'bg-white/40' : 'bg-white/5 border border-white/10'}`}>▼</div>
    <div />
  </div>
);

const VectorBar: React.FC<{ value: number; label: string; positiveLabel: string; negativeLabel: string }> = ({
  value, label, positiveLabel, negativeLabel,
}) => {
  const pct = Math.abs(value) * 50;
  const positive = value >= 0;
  return (
    <div className="flex flex-col gap-1">
      <div className="flex justify-between text-[9px] text-white/40">
        <span>{negativeLabel}</span>
        <span className="text-white/60">{label}</span>
        <span>{positiveLabel}</span>
      </div>
      <div className="relative h-3 bg-white/5 rounded-full border border-white/10 overflow-hidden">
        {/* center line */}
        <div className="absolute left-1/2 top-0 bottom-0 w-px bg-white/20" />
        {/* fill */}
        <div
          className="absolute top-0 bottom-0 rounded-full transition-none"
          style={{
            width: `${pct}%`,
            left: positive ? '50%' : `${50 - pct}%`,
            backgroundColor: '#C400FF',
            opacity: Math.abs(value) > 0.02 ? 1 : 0.2,
          }}
        />
      </div>
      <div className="text-center text-[9px] font-mono text-white/50">
        {value >= 0 ? '+' : ''}{value.toFixed(3)}
      </div>
    </div>
  );
};

// --- main component ---------------------------------------------------------

const XboxControllerStatus: React.FC<Props> = ({ state, paused, onTogglePause }) => {
  const { connected, name, linearX, angularZ, estop } = state;

  return (
    <div className="bg-black/40 border border-white/10 rounded-lg p-4 space-y-4">
      {/* Header row */}
      <div className="flex items-center justify-between gap-3">
        <div className="flex items-center gap-2">
          {/* Controller icon */}
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" className="shrink-0">
            <rect x="2" y="7" width="20" height="10" rx="5" stroke="currentColor" strokeWidth="1.5" className="text-white/60" />
            <circle cx="8" cy="12" r="2" fill={connected ? '#10b981' : 'rgba(255,255,255,0.2)'} />
            <circle cx="16" cy="10" r="1" fill={connected ? 'rgba(255,255,255,0.6)' : 'rgba(255,255,255,0.15)'} />
            <circle cx="18" cy="12" r="1" fill={connected ? 'rgba(255,255,255,0.6)' : 'rgba(255,255,255,0.15)'} />
          </svg>
          <div>
            <div className="text-xs font-semibold text-white/80">
              {connected ? 'Xbox Controller' : 'No Controller'}
            </div>
            {connected && (
              <div className="text-[9px] text-white/35 truncate max-w-[180px]">{name}</div>
            )}
          </div>
          {connected && estop && (
            <span className="text-[10px] px-2 py-0.5 rounded bg-red-500/30 border border-red-400/50 text-red-300 font-bold animate-pulse">
              E-STOP
            </span>
          )}
        </div>

        <button
          onClick={onTogglePause}
          disabled={!connected}
          className={`text-[10px] px-2.5 py-1 rounded border transition-colors ${
            paused
              ? 'bg-amber-500/20 border-amber-400/40 text-amber-200 hover:bg-amber-500/30'
              : 'bg-green-500/20 border-green-400/40 text-green-200 hover:bg-green-500/30'
          } disabled:opacity-30 disabled:cursor-not-allowed`}
          title={paused ? 'Controller paused — click to resume sending' : 'Controller active — click to pause'}
        >
          {paused ? 'Paused' : 'Active'}
        </button>
      </div>

      {!connected ? (
        <div className="text-center py-4 text-white/30 text-xs">
          Plug in an Xbox controller and press any button to activate
        </div>
      ) : (
        <>
          {/* Sticks + Triggers row */}
          <div className="flex items-end justify-center gap-4">
            <TriggerBar value={state.lt} label="LT" color="#00AAFF" />
            <StickViz x={state.leftX} y={state.leftY} label="L-Stick" />
            <StickViz x={state.rightX} y={state.rightY} label="R-Stick" />
            <TriggerBar value={state.rt} label="RT" color="#C400FF" />
          </div>

          {/* Buttons row */}
          <div className="flex items-center justify-between gap-2">
            {/* Shoulder */}
            <div className="flex gap-1.5">
              <Btn active={state.lb} label="LB" color="#00AAFF" />
              <Btn active={state.rb} label="RB" color="#00AAFF" />
            </div>

            {/* D-Pad */}
            <DPad up={state.dUp} down={state.dDown} left={state.dLeft} right={state.dRight} />

            {/* Face buttons */}
            <div className="grid grid-cols-3 gap-1">
              <div />
              <Btn active={state.y} label="Y" color="#FFD700" />
              <div />
              <Btn active={state.x} label="X" color="#4488FF" />
              <div />
              <Btn active={state.b} label="B" color="#FF4444" />
              <div />
              <Btn active={state.a} label="A" color="#44CC44" />
              <div />
            </div>
          </div>

          {/* Drive vector */}
          <div className="space-y-2 pt-1 border-t border-white/5">
            <div className="text-[10px] text-white/40 uppercase tracking-widest">Drive Vector</div>
            <VectorBar
              value={linearX}
              label="Linear X"
              negativeLabel="Reverse"
              positiveLabel="Forward"
            />
            <VectorBar
              value={angularZ}
              label="Angular Z"
              negativeLabel="Right"
              positiveLabel="Left"
            />
          </div>
        </>
      )}
    </div>
  );
};

export default XboxControllerStatus;
