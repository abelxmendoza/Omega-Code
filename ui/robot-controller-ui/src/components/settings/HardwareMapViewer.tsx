/**
 * HardwareMapViewer Component
 * 
 * Read-only display of hardware device mapping.
 */

import React from 'react';
import { Loader2, Cpu, Radio, Camera, Wifi, Zap } from 'lucide-react';
import { useHardwareMap } from '@/hooks/useHardwareMap';
import { ROBOT_ENABLED } from '@/utils/env';

export function HardwareMapViewer() {
  const { hardware, loading, error } = useHardwareMap();

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Hardware map is unavailable.
      </div>
    );
  }

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading hardware map...</span>
      </div>
    );
  }

  if (error || !hardware) {
    return (
      <div className="text-red-400 text-sm">
        Failed to load hardware map: {error?.message || 'Unknown error'}
      </div>
    );
  }

  return (
    <div className="space-y-6">
      {/* GPIO Pins */}
      {hardware.gpio && Object.keys(hardware.gpio).length > 0 && (
        <div>
          <div className="flex items-center gap-2 mb-3">
            <Cpu className="w-5 h-5 text-purple-400" />
            <h3 className="text-lg font-semibold text-white">GPIO Pins</h3>
          </div>
          <div className="grid grid-cols-2 md:grid-cols-3 gap-3">
            {Object.entries(hardware.gpio).map(([name, pin]) => (
              <div
                key={name}
                className="p-3 bg-gray-900/50 rounded border border-gray-700"
              >
                <div className="text-xs text-gray-400 uppercase mb-1">{name.replace(/_/g, ' ')}</div>
                <div className="text-lg font-mono text-white">GPIO {pin}</div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* I2C Devices */}
      {hardware.i2c?.devices && Object.keys(hardware.i2c.devices).length > 0 && (
        <div>
          <div className="flex items-center gap-2 mb-3">
            <Radio className="w-5 h-5 text-purple-400" />
            <h3 className="text-lg font-semibold text-white">I2C Devices</h3>
            {hardware.i2c.bus && (
              <span className="text-sm text-gray-400">Bus {hardware.i2c.bus}</span>
            )}
          </div>
          <div className="space-y-2">
            {Object.entries(hardware.i2c.devices).map(([name, device]: [string, any]) => (
              <div
                key={name}
                className={`p-3 rounded border ${
                  device.enabled !== false
                    ? 'bg-gray-900/50 border-gray-700'
                    : 'bg-gray-900/30 border-gray-800 opacity-50'
                }`}
              >
                <div className="flex items-center justify-between">
                  <div>
                    <div className="font-medium text-white capitalize">{name}</div>
                    {device.description && (
                      <div className="text-xs text-gray-400 mt-1">{device.description}</div>
                    )}
                  </div>
                  <div className="text-right">
                    {device.address && (
                      <div className="text-sm font-mono text-purple-400">0x{device.address}</div>
                    )}
                    {device.enabled === false && (
                      <div className="text-xs text-gray-500 mt-1">Disabled</div>
                    )}
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Camera Devices */}
      {hardware.camera && (
        <div>
          <div className="flex items-center gap-2 mb-3">
            <Camera className="w-5 h-5 text-purple-400" />
            <h3 className="text-lg font-semibold text-white">Camera Devices</h3>
          </div>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
            {hardware.camera.csi && (
              <div className={`p-3 rounded border ${
                hardware.camera.csi.enabled !== false
                  ? 'bg-gray-900/50 border-gray-700'
                  : 'bg-gray-900/30 border-gray-800 opacity-50'
              }`}>
                <div className="font-medium text-white mb-1">CSI Camera</div>
                {hardware.camera.csi.device && (
                  <div className="text-sm font-mono text-gray-400">{hardware.camera.csi.device}</div>
                )}
                {hardware.camera.csi.description && (
                  <div className="text-xs text-gray-500 mt-1">{hardware.camera.csi.description}</div>
                )}
              </div>
            )}
            {hardware.camera.usb && (
              <div className={`p-3 rounded border ${
                hardware.camera.usb.enabled !== false
                  ? 'bg-gray-900/50 border-gray-700'
                  : 'bg-gray-900/30 border-gray-800 opacity-50'
              }`}>
                <div className="font-medium text-white mb-1">USB Camera</div>
                {hardware.camera.usb.device && (
                  <div className="text-sm font-mono text-gray-400">{hardware.camera.usb.device}</div>
                )}
                {hardware.camera.usb.description && (
                  <div className="text-xs text-gray-500 mt-1">{hardware.camera.usb.description}</div>
                )}
              </div>
            )}
          </div>
        </div>
      )}

      {/* Network Interfaces */}
      {hardware.network && (
        <div>
          <div className="flex items-center gap-2 mb-3">
            <Wifi className="w-5 h-5 text-purple-400" />
            <h3 className="text-lg font-semibold text-white">Network Interfaces</h3>
          </div>
          <div className="space-y-2">
            {hardware.network.wifi && (
              <div className="p-3 bg-gray-900/50 rounded border border-gray-700">
                <div className="font-medium text-white mb-1">Wi-Fi</div>
                <div className="text-sm text-gray-400">Interface: {hardware.network.wifi.interface || 'wlan0'}</div>
              </div>
            )}
            {hardware.network.bluetooth && (
              <div className={`p-3 rounded border ${
                hardware.network.bluetooth.enabled !== false
                  ? 'bg-gray-900/50 border-gray-700'
                  : 'bg-gray-900/30 border-gray-800 opacity-50'
              }`}>
                <div className="font-medium text-white mb-1">Bluetooth</div>
                <div className="text-sm text-gray-400">
                  Interface: {hardware.network.bluetooth.interface || 'hci0'}
                  {hardware.network.bluetooth.enabled === false && (
                    <span className="ml-2 text-gray-500">(Disabled)</span>
                  )}
                </div>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Power Management */}
      {hardware.power && (
        <div>
          <div className="flex items-center gap-2 mb-3">
            <Zap className="w-5 h-5 text-purple-400" />
            <h3 className="text-lg font-semibold text-white">Power Management</h3>
          </div>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
            {hardware.power.voltage_monitor && (
              <div className={`p-3 rounded border ${
                hardware.power.voltage_monitor.enabled !== false
                  ? 'bg-gray-900/50 border-gray-700'
                  : 'bg-gray-900/30 border-gray-800 opacity-50'
              }`}>
                <div className="font-medium text-white mb-1">Voltage Monitor</div>
                {hardware.power.voltage_monitor.pin !== null && (
                  <div className="text-sm text-gray-400">GPIO {hardware.power.voltage_monitor.pin}</div>
                )}
              </div>
            )}
            {hardware.power.battery_monitor && (
              <div className={`p-3 rounded border ${
                hardware.power.battery_monitor.enabled !== false
                  ? 'bg-gray-900/50 border-gray-700'
                  : 'bg-gray-900/30 border-gray-800 opacity-50'
              }`}>
                <div className="font-medium text-white mb-1">Battery Monitor</div>
                {hardware.power.battery_monitor.i2c_address && (
                  <div className="text-sm font-mono text-gray-400">0x{hardware.power.battery_monitor.i2c_address}</div>
                )}
              </div>
            )}
          </div>
        </div>
      )}

      {/* Validation Info */}
      {hardware.validation && (
        <div className="p-4 bg-blue-900/20 rounded border border-blue-500/30">
          <h4 className="text-sm font-semibold text-blue-300 mb-2">Hardware Requirements</h4>
          {hardware.validation.required_devices && hardware.validation.required_devices.length > 0 && (
            <div className="mb-2">
              <div className="text-xs text-blue-400 mb-1">Required:</div>
              <div className="flex flex-wrap gap-2">
                {hardware.validation.required_devices.map((device) => (
                  <span key={device} className="px-2 py-1 bg-blue-500/20 rounded text-xs text-blue-300">
                    {device}
                  </span>
                ))}
              </div>
            </div>
          )}
          {hardware.validation.optional_devices && hardware.validation.optional_devices.length > 0 && (
            <div>
              <div className="text-xs text-blue-400 mb-1">Optional:</div>
              <div className="flex flex-wrap gap-2">
                {hardware.validation.optional_devices.map((device) => (
                  <span key={device} className="px-2 py-1 bg-gray-700/50 rounded text-xs text-gray-400">
                    {device}
                  </span>
                ))}
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

