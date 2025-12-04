/**
 * useRobotOnline Hook
 * Provides a clean way to check if robot backend is available
 */

import { ROBOT_ENABLED } from "@/utils/env";

export function useRobotOnline() {
  return ROBOT_ENABLED;
}

