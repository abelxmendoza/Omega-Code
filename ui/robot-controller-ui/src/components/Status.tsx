/* 
# File: /src/components/Status.tsx
# Summary:
Small drive-ready indicator + battery bar. No sockets — props only.
*/

import React from 'react';
import { FaCheckCircle, FaTimesCircle } from 'react-icons/fa';

type Props = {
  connected?: boolean;     // "drive-ready"
  batteryLevel?: number;   // 0-100
  battery?: number;        // 0-100 (for test compatibility)
  status?: string;        // "Connected" | "Disconnected" (for test compatibility)
  upCount?: number;        // optional: services up
  total?: number;          // optional: total services
  className?: string;
};

const Status: React.FC<Props> = ({ 
  connected = false, 
  batteryLevel = 0, 
  battery, 
  status,
  upCount, 
  total, 
  className = '' 
}) => {
  // Use battery prop if provided (for tests), otherwise batteryLevel
  const actualBattery = battery !== undefined ? battery : batteryLevel;
  const actualConnected = status ? status === 'Connected' : connected;
  
  const batteryClass =
    actualBattery > 75 ? 'bg-green-500'
  : actualBattery > 50 ? 'bg-yellow-500'
  : actualBattery > 20 ? 'bg-blue-500 neon-blue'
  : 'bg-red-500';

  return (
    <div className={`flex items-center gap-3 ${className}`}>
      {/* compact drive-ready chip */}
      <div className="flex items-center gap-1 text-sm">
        {actualConnected ? (
          <FaCheckCircle className="text-green-500" title="Drive ready" />
        ) : (
          <FaTimesCircle className="text-red-500" title="Not ready" />
        )}
        {typeof upCount === 'number' && typeof total === 'number' && (
          <span className="text-xs text-white/70">{upCount}/{total}</span>
        )}
      </div>

      {/* battery */}
      <div className="flex items-center gap-2">
        <div className="w-28 h-3 bg-white/10 rounded overflow-hidden">
          <div className={`h-full ${batteryClass}`} style={{ width: `${actualBattery}%` }} />
        </div>
        <span className="text-xs text-white/80 w-10 text-right">{actualBattery}%</span>
      </div>
    </div>
  );
};

export default Status;
