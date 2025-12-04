/**
 * Robot Offline Banner Component
 * Displays a banner when robot backend is unavailable (e.g., on Vercel)
 */

import { ROBOT_ENABLED } from "@/utils/env";

export default function RobotOfflineBanner() {
  if (ROBOT_ENABLED) return null;

  return (
    <div className="bg-red-900 text-red-200 text-center py-2 text-sm">
      ⚠️ Robot backend inactive — control systems disabled
    </div>
  );
}

