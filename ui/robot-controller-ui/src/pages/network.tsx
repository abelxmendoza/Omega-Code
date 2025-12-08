/**
 * Network Management Page
 * 
 * Provides comprehensive network management interface for
 * WiFi, Tailscale, and mobile hotspot connections.
 */

import React from 'react';
import Head from 'next/head';
import dynamic from 'next/dynamic';
import NetworkProfileSelector from '@/components/NetworkProfileSelector';
import MobileConnectionTest from '@/components/MobileConnectionTest';

const OmegaNetworkWizard = dynamic(
  () => import('@/components/network/OmegaNetworkWizard'),
  { ssr: false }
);

export default function NetworkPage() {
  return (
    <>
      <Head>
        <title>Network Management - Robot Controller</title>
        <meta name="description" content="Manage network connections and optimize performance" />
      </Head>

      <div className="min-h-screen bg-gray-50 p-4">
        <div className="max-w-6xl mx-auto space-y-6">
          {/* Header */}
          <div className="bg-white rounded-lg shadow-sm border p-6">
            <div className="flex items-center justify-between">
              <div>
                <h1 className="text-2xl font-bold text-gray-900">Network Management</h1>
                <p className="text-gray-600 mt-1">
                  Manage network profiles and optimize connection performance
                </p>
              </div>
              <div className="text-sm text-gray-500">
                Last updated: {new Date().toLocaleTimeString()}
              </div>
            </div>
          </div>

          {/* Omega Network Wizard */}
          <OmegaNetworkWizard />

          {/* Network Profile Selector */}
          <NetworkProfileSelector 
            className="bg-white"
            onProfileChange={(profile) => {
              console.log('[NetworkPage] Profile changed:', profile.name);
            }}
          />

          {/* Connection Testing */}
          <MobileConnectionTest 
            className="bg-white"
            onTestComplete={(result) => {
              console.log('[NetworkPage] Test completed:', result);
            }}
          />

          {/* Network Information */}
          <div className="bg-white rounded-lg shadow-sm border p-6">
            <h2 className="text-lg font-semibold text-gray-900 mb-4">Network Information</h2>
            
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              {/* Connection Types */}
              <div>
                <h3 className="text-sm font-medium text-gray-700 mb-3">Supported Connection Types</h3>
                <div className="space-y-2">
                  <div className="flex items-center gap-2 text-sm">
                    <span className="text-lg">🔒</span>
                    <span className="text-gray-900">Tailscale VPN</span>
                    <span className="text-xs bg-green-100 text-green-800 px-2 py-1 rounded">Recommended</span>
                  </div>
                  <div className="flex items-center gap-2 text-sm">
                    <span className="text-lg">📶</span>
                    <span className="text-gray-900">WiFi Network</span>
                    <span className="text-xs bg-blue-100 text-blue-800 px-2 py-1 rounded">Fast</span>
                  </div>
                  <div className="flex items-center gap-2 text-sm">
                    <span className="text-lg">📱</span>
                    <span className="text-gray-900">Mobile Hotspot</span>
                    <span className="text-xs bg-yellow-100 text-yellow-800 px-2 py-1 rounded">Optimized</span>
                  </div>
                  <div className="flex items-center gap-2 text-sm">
                    <span className="text-lg">🔗</span>
                    <span className="text-gray-900">Direct Connection</span>
                    <span className="text-xs bg-gray-100 text-gray-800 px-2 py-1 rounded">Fallback</span>
                  </div>
                </div>
              </div>

              {/* Optimization Features */}
              <div>
                <h3 className="text-sm font-medium text-gray-700 mb-3">Optimization Features</h3>
                <div className="space-y-2 text-sm text-gray-600">
                  <div className="flex items-center gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Automatic profile selection</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Connection quality monitoring</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Automatic fallback</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Mobile hotspot optimization</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Bandwidth-aware video streaming</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-green-500">✓</span>
                    <span>Real-time performance metrics</span>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Usage Tips */}
          <div className="bg-blue-50 rounded-lg border border-blue-200 p-6">
            <h2 className="text-lg font-semibold text-blue-900 mb-4">Usage Tips</h2>
            
            <div className="space-y-4 text-sm text-blue-800">
              <div>
                <h3 className="font-medium mb-2">For Mobile Hotspot:</h3>
                <ul className="list-disc list-inside space-y-1 ml-4">
                  <li>System automatically optimizes for cellular networks</li>
                  <li>Video quality is reduced to save bandwidth</li>
                  <li>Connection timeouts are increased for stability</li>
                  <li>Compression is enabled to reduce data usage</li>
                </ul>
              </div>
              
              <div>
                <h3 className="font-medium mb-2">For Tailscale:</h3>
                <ul className="list-disc list-inside space-y-1 ml-4">
                  <li>Best performance and security</li>
                  <li>Works through firewalls and NAT</li>
                  <li>No need for port forwarding</li>
                  <li>Automatic reconnection if connection drops</li>
                </ul>
              </div>
              
              <div>
                <h3 className="font-medium mb-2">For WiFi:</h3>
                <ul className="list-disc list-inside space-y-1 ml-4">
                  <li>High-quality video streaming</li>
                  <li>Low latency for real-time control</li>
                  <li>Fast data transfer rates</li>
                  <li>Stable connection for extended use</li>
                </ul>
              </div>
            </div>
          </div>

          {/* Troubleshooting */}
          <div className="bg-yellow-50 rounded-lg border border-yellow-200 p-6">
            <h2 className="text-lg font-semibold text-yellow-900 mb-4">Troubleshooting</h2>
            
            <div className="space-y-3 text-sm text-yellow-800">
              <div>
                <h3 className="font-medium">Connection Issues:</h3>
                <p className="ml-4">Use the &quot;Fallback&quot; button to switch to the next available network profile</p>
              </div>
              
              <div>
                <h3 className="font-medium">Poor Performance:</h3>
                <p className="ml-4">Run the connection test to get optimization recommendations</p>
              </div>
              
              <div>
                <h3 className="font-medium">Mobile Hotspot Problems:</h3>
                <p className="ml-4">Ensure your mobile device has a strong cellular signal and sufficient data allowance</p>
              </div>
              
              <div>
                <h3 className="font-medium">Tailscale Issues:</h3>
                <p className="ml-4">Check that Tailscale is running on both your device and the robot</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
