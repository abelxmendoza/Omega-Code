/**
 * usePersistentState Hook
 * 
 * Fetches and updates persistent runtime state.
 */

import { useState, useEffect, useCallback } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

interface UsePersistentStateReturn {
  state: Record<string, any> | null;
  loading: boolean;
  error: Error | null;
  updateState: (key: string, value: any) => Promise<boolean>;
  refresh: () => Promise<void>;
}

export function usePersistentState(): UsePersistentStateReturn {
  const [state, setState] = useState<Record<string, any> | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const fetchState = useCallback(async () => {
    if (!ROBOT_ENABLED) {
      setState(null);
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch('/api/config/state');
      
      if (response.offline) {
        setState(null);
        setError(null);
        return;
      }

      const data = await response.json();
      
      if (data.ok && data.state) {
        setState(data.state);
        setError(null);
      } else {
        throw new Error('Invalid response format');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to fetch state');
      setError(error);
      console.error('Failed to fetch persistent state:', error);
    } finally {
      setLoading(false);
    }
  }, []);

  const updateState = useCallback(async (key: string, value: any): Promise<boolean> => {
    if (!ROBOT_ENABLED) {
      return false;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch(`/api/config/state/${key}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(value),
      });
      
      if (response.offline) {
        return false;
      }

      const result = await response.json();
      
      if (result.ok) {
        // Update local state
        if (state) {
          setState({ ...state, [key]: value });
        }
        setError(null);
        return true;
      } else {
        throw new Error(result.error || 'Failed to update state');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to update state');
      setError(error);
      console.error(`Failed to update state key ${key}:`, error);
      return false;
    } finally {
      setLoading(false);
    }
  }, [state]);

  useEffect(() => {
    fetchState();
  }, [fetchState]);

  return {
    state,
    loading,
    error,
    updateState,
    refresh: fetchState,
  };
}

