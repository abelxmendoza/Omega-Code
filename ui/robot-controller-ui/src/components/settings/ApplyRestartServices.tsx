/**
 * ApplyRestartServices Component
 * 
 * Validates config and restarts orchestrator.
 */

import React, { useState } from 'react';
import { RefreshCw, CheckCircle, AlertCircle, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';
import { useConfig } from '@/hooks/useConfig';

export function ApplyRestartServices() {
  const { refresh } = useConfig();
  const [validating, setValidating] = useState(false);
  const [restarting, setRestarting] = useState(false);
  const [validationResult, setValidationResult] = useState<{
    valid: boolean;
    errors: string[];
  } | null>(null);
  const [restartResult, setRestartResult] = useState<{
    success: boolean;
    message: string;
  } | null>(null);

  const validateConfig = async () => {
    if (!ROBOT_ENABLED) return;

    setValidating(true);
    setValidationResult(null);
    setRestartResult(null);

    try {
      const response = await robotFetch('/api/config/validate');
      
      if (response.offline) {
        setValidationResult({
          valid: false,
          errors: ['Robot is offline'],
        });
        return;
      }

      const data = await response.json();
      setValidationResult({
        valid: data.valid || data.ok,
        errors: data.errors || [],
      });
    } catch (error) {
      setValidationResult({
        valid: false,
        errors: [error instanceof Error ? error.message : 'Validation failed'],
      });
      console.error('Failed to validate config:', error);
    } finally {
      setValidating(false);
    }
  };

  const restartOrchestrator = async () => {
    if (!ROBOT_ENABLED || restarting) return;

    // First validate
    await validateConfig();
    
    if (validationResult && !validationResult.valid) {
      setRestartResult({
        success: false,
        message: 'Configuration is invalid. Please fix errors before restarting.',
      });
      return;
    }

    setRestarting(true);
    setRestartResult(null);

    try {
      // Note: This endpoint would need to be created in the backend
      // For now, we'll use a service restart approach
      const response = await robotFetch('/api/services/restart/main_api', {
        method: 'POST',
      });
      
      if (response.offline) {
        setRestartResult({
          success: false,
          message: 'Robot is offline',
        });
        return;
      }

      const data = await response.json();
      
      if (data.ok) {
        setRestartResult({
          success: true,
          message: 'Orchestrator restarted successfully. Services will reload configuration.',
        });
        // Refresh config after restart
        setTimeout(() => {
          refresh();
        }, 2000);
      } else {
        setRestartResult({
          success: false,
          message: data.error || 'Failed to restart orchestrator',
        });
      }
    } catch (error) {
      setRestartResult({
        success: false,
        message: error instanceof Error ? error.message : 'Restart failed',
      });
      console.error('Failed to restart orchestrator:', error);
    } finally {
      setRestarting(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Service restart is disabled.
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {/* Validation Result */}
      {validationResult && (
        <div
          className={`p-4 rounded border ${
            validationResult.valid
              ? 'bg-green-900/20 border-green-500/50'
              : 'bg-red-900/20 border-red-500/50'
          }`}
        >
          <div className="flex items-start gap-2">
            {validationResult.valid ? (
              <CheckCircle className="w-5 h-5 text-green-400 flex-shrink-0 mt-0.5" />
            ) : (
              <AlertCircle className="w-5 h-5 text-red-400 flex-shrink-0 mt-0.5" />
            )}
            <div className="flex-1">
              <div
                className={`font-semibold mb-1 ${
                  validationResult.valid ? 'text-green-300' : 'text-red-300'
                }`}
              >
                {validationResult.valid ? 'Configuration Valid' : 'Configuration Invalid'}
              </div>
              {validationResult.errors.length > 0 && (
                <div className="text-sm text-red-300 space-y-1">
                  {validationResult.errors.map((error, i) => (
                    <div key={i}>• {error}</div>
                  ))}
                </div>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Restart Result */}
      {restartResult && (
        <div
          className={`p-4 rounded border ${
            restartResult.success
              ? 'bg-green-900/20 border-green-500/50'
              : 'bg-red-900/20 border-red-500/50'
          }`}
        >
          <div className="flex items-start gap-2">
            {restartResult.success ? (
              <CheckCircle className="w-5 h-5 text-green-400 flex-shrink-0 mt-0.5" />
            ) : (
              <AlertCircle className="w-5 h-5 text-red-400 flex-shrink-0 mt-0.5" />
            )}
            <div className={`text-sm ${restartResult.success ? 'text-green-300' : 'text-red-300'}`}>
              {restartResult.message}
            </div>
          </div>
        </div>
      )}

      {/* Actions */}
      <div className="flex gap-3">
        <Button
          onClick={validateConfig}
          disabled={validating || restarting}
          variant="outline"
          className="border-gray-600 text-gray-300 hover:bg-gray-700"
        >
          {validating ? (
            <>
              <Loader2 className="w-4 h-4 mr-2 animate-spin" />
              Validating...
            </>
          ) : (
            <>
              <CheckCircle className="w-4 h-4 mr-2" />
              Validate Config
            </>
          )}
        </Button>

        <Button
          onClick={restartOrchestrator}
          disabled={restarting || validating}
          className="bg-purple-600 hover:bg-purple-700 text-white"
        >
          {restarting ? (
            <>
              <Loader2 className="w-4 h-4 mr-2 animate-spin" />
              Restarting...
            </>
          ) : (
            <>
              <RefreshCw className="w-4 h-4 mr-2" />
              Apply & Restart Services
            </>
          )}
        </Button>
      </div>

      <div className="text-xs text-gray-400 p-3 bg-gray-900/50 rounded border border-gray-700">
        <strong>Note:</strong> Restarting the orchestrator will reload all configuration and restart
        services according to their autostart settings. Any unsaved changes will be lost.
      </div>
    </div>
  );
}

