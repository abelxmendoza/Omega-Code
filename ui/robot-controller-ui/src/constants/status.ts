// src/constants/status.ts
export type ServerStatus = 'connecting' | 'connected' | 'disconnected';

export const statusColor = {
  connected: 'bg-emerald-500',
  connecting: 'bg-amber-400',
  disconnected: 'bg-rose-500',
} as const;
