/**
 * ServiceTable Component
 * 
 * Displays a table of all services with their status, health, and actions.
 */

import React from 'react';
import { Play, Square, RotateCw, CheckCircle, XCircle, AlertCircle, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { useServiceStatus, ServiceStatus } from '@/hooks/useServiceStatus';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

interface ServiceTableProps {
  onServiceSelect?: (serviceName: string) => void;
  onActionComplete?: () => void;
}

export function ServiceTable({ onServiceSelect, onActionComplete }: ServiceTableProps) {
  const { services, loading, refresh } = useServiceStatus({ interval: 2000 });
  const [actionLoading, setActionLoading] = React.useState<string | null>(null);

  const performAction = async (action: 'start' | 'stop' | 'restart', name: string) => {
    if (!ROBOT_ENABLED) return;

    setActionLoading(name);
    try {
      const response = await robotFetch(`/api/services/${action}/${name}`, {
        method: 'POST',
      });
      
      if (response.offline) return;
      
      const data = await response.json();
      if (data.ok) {
        await refresh();
        onActionComplete?.();
      }
    } catch (error) {
      console.error(`Failed to ${action} service ${name}:`, error);
    } finally {
      setActionLoading(null);
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'running':
        return <CheckCircle className="w-5 h-5 text-green-400" />;
      case 'stopped':
        return <XCircle className="w-5 h-5 text-gray-400" />;
      case 'crashed':
        return <AlertCircle className="w-5 h-5 text-red-400" />;
      case 'starting':
      case 'stopping':
        return <Loader2 className="w-5 h-5 text-yellow-400 animate-spin" />;
      default:
        return <AlertCircle className="w-5 h-5 text-gray-400" />;
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'running':
        return 'text-green-400';
      case 'stopped':
        return 'text-gray-400';
      case 'crashed':
        return 'text-red-400';
      case 'starting':
        return 'text-yellow-400';
      case 'stopping':
        return 'text-orange-400';
      default:
        return 'text-gray-400';
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-center py-8 text-gray-400">
        Robot is offline. Service management features are disabled.
      </div>
    );
  }

  if (loading && services.length === 0) {
    return (
      <div className="text-center py-8">
        <Loader2 className="w-8 h-8 animate-spin mx-auto text-gray-400" />
        <p className="text-gray-400 mt-2">Loading services...</p>
      </div>
    );
  }

  return (
    <div className="overflow-x-auto">
      <table className="w-full border-collapse">
        <thead>
          <tr className="border-b border-gray-700">
            <th className="text-left p-3 text-sm font-semibold text-gray-300">Service</th>
            <th className="text-left p-3 text-sm font-semibold text-gray-300">Status</th>
            <th className="text-left p-3 text-sm font-semibold text-gray-300">Health</th>
            <th className="text-left p-3 text-sm font-semibold text-gray-300">PID</th>
            <th className="text-left p-3 text-sm font-semibold text-gray-300">Policy</th>
            <th className="text-left p-3 text-sm font-semibold text-gray-300">Actions</th>
          </tr>
        </thead>
        <tbody>
          {services.map((service) => (
            <tr
              key={service.name}
              className="border-b border-gray-800 hover:bg-gray-800/50 transition-colors"
            >
              <td className="p-3">
                <div>
                  <div className="font-medium text-white">{service.display_name}</div>
                  <div className="text-xs text-gray-400">{service.description}</div>
                  <div className="flex gap-2 mt-1">
                    {service.autostart && (
                      <Badge variant="outline" className="text-xs border-blue-500 text-blue-400">
                        Autostart
                      </Badge>
                    )}
                    <Badge variant="outline" className="text-xs border-gray-600 text-gray-400">
                      {service.type}
                    </Badge>
                    {service.port && (
                      <Badge variant="outline" className="text-xs border-gray-600 text-gray-400">
                        :{service.port}
                      </Badge>
                    )}
                  </div>
                </div>
              </td>
              <td className="p-3">
                <div className="flex items-center gap-2">
                  {getStatusIcon(service.status)}
                  <span className={`text-sm font-medium ${getStatusColor(service.status)}`}>
                    {service.status.toUpperCase()}
                  </span>
                </div>
              </td>
              <td className="p-3">
                {service.health ? (
                  <Badge
                    variant={service.health.healthy ? 'default' : 'destructive'}
                    className={service.health.healthy ? 'bg-green-600' : 'bg-red-600'}
                  >
                    {service.health.healthy ? 'Healthy' : 'Unhealthy'}
                  </Badge>
                ) : (
                  <span className="text-xs text-gray-500">N/A</span>
                )}
              </td>
              <td className="p-3">
                {service.pid ? (
                  <span className="text-sm font-mono text-gray-300">{service.pid}</span>
                ) : (
                  <span className="text-xs text-gray-500">-</span>
                )}
              </td>
              <td className="p-3">
                <Badge variant="outline" className="text-xs border-purple-500 text-purple-400">
                  {service.restart_policy}
                </Badge>
              </td>
              <td className="p-3">
                <div className="flex gap-2">
                  {service.status === 'running' ? (
                    <>
                      <Button
                        size="sm"
                        variant="outline"
                        onClick={() => performAction('stop', service.name)}
                        disabled={actionLoading === service.name}
                        className="border-gray-600 text-gray-300 hover:bg-gray-700"
                      >
                        {actionLoading === service.name ? (
                          <Loader2 className="w-4 h-4 animate-spin" />
                        ) : (
                          <>
                            <Square className="w-4 h-4 mr-1" />
                            Stop
                          </>
                        )}
                      </Button>
                      <Button
                        size="sm"
                        variant="outline"
                        onClick={() => performAction('restart', service.name)}
                        disabled={actionLoading === service.name}
                        className="border-gray-600 text-gray-300 hover:bg-gray-700"
                      >
                        {actionLoading === service.name ? (
                          <Loader2 className="w-4 h-4 animate-spin" />
                        ) : (
                          <>
                            <RotateCw className="w-4 h-4 mr-1" />
                            Restart
                          </>
                        )}
                      </Button>
                    </>
                  ) : (
                    <Button
                      size="sm"
                      variant="outline"
                      onClick={() => performAction('start', service.name)}
                      disabled={actionLoading === service.name}
                      className="border-gray-600 text-gray-300 hover:bg-gray-700"
                    >
                      {actionLoading === service.name ? (
                        <Loader2 className="w-4 h-4 animate-spin" />
                      ) : (
                        <>
                          <Play className="w-4 h-4 mr-1" />
                          Start
                        </>
                      )}
                    </Button>
                  )}
                  {onServiceSelect && (
                    <Button
                      size="sm"
                      variant="outline"
                      onClick={() => onServiceSelect(service.name)}
                      className="border-gray-600 text-gray-300 hover:bg-gray-700"
                    >
                      Logs
                    </Button>
                  )}
                </div>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {services.length === 0 && !loading && (
        <div className="text-center py-8 text-gray-400">
          No services found.
        </div>
      )}
    </div>
  );
}

