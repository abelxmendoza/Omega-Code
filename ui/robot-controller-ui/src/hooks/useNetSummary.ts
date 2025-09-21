/*
# File: /Omega-Code/ui/robot-controller-ui/src/hooks/useNetSummary.ts
# Summary: Polls /api/net/summary for live network state.
*/

import * as React from 'react';
import { netApi, NetSummary } from '@/utils/netApi';

export function useNetSummary(intervalMs = 7000) {
  const [data, setData] = React.useState<NetSummary | null>(null);
  const [error, setError] = React.useState<string | null>(null);
  const [loading, setLoading] = React.useState(true);

  const load = React.useCallback(async () => {
    try {
      const res = await netApi.summary();
      setData(res);
      setError(null);
    } catch (e: any) {
      setError(e?.message ?? 'summary failed');
    } finally {
      setLoading(false);
    }
  }, []);

  React.useEffect(() => {
    let t: ReturnType<typeof setInterval> | null = null;
    load();
    t = setInterval(load, intervalMs);
    return () => { if (t) clearInterval(t); };
  }, [load, intervalMs]);

  return { data, error, loading, refresh: load };
}
