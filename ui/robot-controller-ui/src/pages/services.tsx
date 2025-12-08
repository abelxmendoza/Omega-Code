/**
 * Service Management Page
 * 
 * Provides UI for managing OmegaOS services:
 * - List all services
 * - Start/Stop/Restart services
 * - View service status and health
 * - View service logs
 */

import React, { useState } from 'react';
import Head from 'next/head';
import { RefreshCw } from 'lucide-react';
import { ROBOT_ENABLED } from '@/utils/env';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { ServiceTable } from '@/components/services/ServiceTable';
import { ServiceLogs } from '@/components/services/ServiceLogs';
import { useServiceStatus } from '@/hooks/useServiceStatus';

export default function ServicesPage() {
  const { services, loading, refresh } = useServiceStatus({ interval: 2000 });
  const [selectedService, setSelectedService] = useState<string | null>(null);
  
  const selectedServiceData = services.find(s => s.name === selectedService);

  if (!ROBOT_ENABLED) {
    return (
      <div className="min-h-screen bg-gray-900 text-white p-4">
        <Head>
          <title>Service Management - Robot Controller</title>
        </Head>
        <Card className="bg-gray-800 border-gray-700">
          <CardContent className="p-6">
            <p className="text-gray-400">Robot is offline. Service management features are disabled.</p>
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <>
      <Head>
        <title>Service Management - Robot Controller</title>
        <meta name="description" content="Manage OmegaOS services" />
      </Head>

      <div className="min-h-screen bg-gray-900 text-white p-4">
        <div className="max-w-7xl mx-auto space-y-6">
          {/* Header */}
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold text-white">Service Management</h1>
              <p className="text-gray-400 mt-1">Manage OmegaOS robot services</p>
            </div>
            <Button
              onClick={refresh}
              disabled={loading}
              variant="outline"
              className="border-gray-600 text-gray-300 hover:bg-gray-700"
            >
              <RefreshCw className={`w-4 h-4 mr-2 ${loading ? 'animate-spin' : ''}`} />
              Refresh
            </Button>
          </div>

          {/* Service Table */}
          <Card className="bg-gray-800 border-gray-700">
            <CardHeader>
              <CardTitle className="text-xl text-white">Services</CardTitle>
            </CardHeader>
            <CardContent>
              <ServiceTable
                onServiceSelect={setSelectedService}
                onActionComplete={refresh}
              />
            </CardContent>
          </Card>

          {/* Logs Panel */}
          {selectedService && selectedServiceData && (
            <Card className="bg-gray-800 border-gray-700">
              <CardContent className="p-6">
                <ServiceLogs
                  serviceName={selectedService}
                  serviceDisplayName={selectedServiceData.display_name}
                  onClose={() => setSelectedService(null)}
                  autoRefresh={true}
                  refreshInterval={10000}
                />
              </CardContent>
            </Card>
          )}

          {/* Empty State */}
          {!loading && services.length === 0 && (
            <Card className="bg-gray-800 border-gray-700">
              <CardContent className="p-6 text-center">
                <p className="text-gray-400">No services found or robot is offline.</p>
              </CardContent>
            </Card>
          )}
        </div>
      </div>
    </>
  );
}
