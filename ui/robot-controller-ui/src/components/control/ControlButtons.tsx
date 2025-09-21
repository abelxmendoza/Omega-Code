/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/control/ControlButtons.tsx
# Summary:
Start / Stop / Apply row with sane defaults + accessibility.
- If callbacks aren’t provided, it uses CommandContext defaults:
  • Start → send {command:"status"} (harmless probe)
  • Stop  → send {command:"move-stop"} (emergency halt)
  • Apply → no-op (can be wired by parent)
*/

import React from 'react';
import { useCommand } from '@/context/CommandContext';
import { COMMAND } from '@/control_definitions';

type Props = {
  onStart?: () => void;
  onStop?: () => void;
  onApply?: () => void;
  disabled?: boolean;
  busy?: boolean; // optional spinner state from parent
};

export default function ControlButtons({
  onStart,
  onStop,
  onApply,
  disabled = false,
  busy = false,
}: Props) {
  const { sendCommand, addCommand } = useCommand();

  const safeSend = React.useCallback((cmd: string) => {
    try {
      sendCommand(cmd);
      addCommand(`Sent: ${cmd}`);
    } catch (e) {
      addCommand(`Send failed: ${String(e)}`);
    }
  }, [sendCommand, addCommand]);

  const doStart = onStart ?? (() => safeSend('status'));
  const doStop  = onStop  ?? (() => safeSend(COMMAND.MOVE_STOP || 'move-stop'));
  const doApply = onApply ?? (() => { /* parent can supply; keep as no-op */ });

  const common =
    'px-4 py-2 rounded text-white font-semibold disabled:opacity-50 disabled:cursor-not-allowed';

  return (
    <div className="flex space-x-3 items-center">
      <button
        className={`${common} bg-emerald-600 hover:bg-emerald-500`}
        onClick={doStart}
        disabled={disabled || busy}
        aria-busy={busy}
        aria-label="Start / Status probe"
        title="Start / Status"
      >
        {busy ? 'Working…' : 'Start'}
      </button>

      <button
        className={`${common} bg-rose-600 hover:bg-rose-500`}
        onClick={doStop}
        disabled={disabled}
        aria-label="Emergency Stop"
        title="Stop motors"
      >
        Stop
      </button>

      <button
        className={`${common} bg-sky-600 hover:bg-sky-500`}
        onClick={doApply}
        disabled={disabled || busy}
        aria-label="Apply Settings"
        title="Apply settings"
      >
        Apply
      </button>
    </div>
  );
}
