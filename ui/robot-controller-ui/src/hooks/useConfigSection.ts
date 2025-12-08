/**
 * useConfigSection Hook
 * 
 * Fetches and updates specific configuration sections.
 */

import { useState, useCallback } from 'react';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

interface UseConfigSectionReturn {
  section: Record<string, any> | null;
  loading: boolean;
  error: Error | null;
  updateSection: (data: Record<string, any>) => Promise<boolean>;
  refresh: () => Promise<void>;
}

export function useConfigSection(sectionName: string): UseConfigSectionReturn {
  const [section, setSection] = useState<Record<string, any> | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<Error | null>(null);

  const fetchSection = useCallback(async () => {
    if (!ROBOT_ENABLED) {
      setSection(null);
      setError(null);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch(`/api/config/${sectionName}`);
      
      if (response.offline) {
        setSection(null);
        setError(null);
        return;
      }

      const data = await response.json();
      
      if (data.ok && data.data) {
        setSection(data.data);
        setError(null);
      } else {
        throw new Error(data.error || 'Failed to fetch section');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to fetch section');
      setError(error);
      console.error(`Failed to fetch config section ${sectionName}:`, error);
    } finally {
      setLoading(false);
    }
  }, [sectionName]);

  const updateSection = useCallback(async (data: Record<string, any>): Promise<boolean> => {
    if (!ROBOT_ENABLED) {
      return false;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch(`/api/config/${sectionName}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
      });
      
      if (response.offline) {
        return false;
      }

      const result = await response.json();
      
      if (result.ok) {
        setSection(result.data);
        setError(null);
        return true;
      } else {
        throw new Error(result.error || 'Failed to update section');
      }
    } catch (err) {
      const error = err instanceof Error ? err : new Error('Failed to update section');
      setError(error);
      console.error(`Failed to update config section ${sectionName}:`, error);
      return false;
    } finally {
      setLoading(false);
    }
  }, [sectionName]);

  return {
    section,
    loading,
    error,
    updateSection,
    refresh: fetchSection,
  };
}

