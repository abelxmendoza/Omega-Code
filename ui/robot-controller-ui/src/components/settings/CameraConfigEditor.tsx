/**
 * CameraConfigEditor Component
 * 
 * Edits camera configuration (backend, resolution, FPS).
 */

import React, { useState, useEffect } from 'react';
import { Save, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { useConfigSection } from '@/hooks/useConfigSection';
import { validateCameraBackend, validateCameraResolution, validateCameraFPS } from '@/utils/validators';
import { ROBOT_ENABLED } from '@/utils/env';

export function CameraConfigEditor() {
  const { section, loading, updateSection } = useConfigSection('camera');
  const [saving, setSaving] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});

  const [backend, setBackend] = useState('picamera2');
  const [device, setDevice] = useState('/dev/video0');
  const [width, setWidth] = useState(640);
  const [height, setHeight] = useState(480);
  const [fps, setFps] = useState(30);
  const [testMode, setTestMode] = useState(false);

  useEffect(() => {
    if (section) {
      setBackend(section.backend || 'picamera2');
      setDevice(section.device || '/dev/video0');
      setWidth(section.width || 640);
      setHeight(section.height || 480);
      setFps(section.fps || 30);
      setTestMode(section.test_mode || false);
    }
  }, [section]);

  const validate = (): boolean => {
    const newErrors: Record<string, string> = {};

    const backendValidation = validateCameraBackend(backend);
    if (!backendValidation.valid) {
      newErrors.backend = backendValidation.errors[0];
    }

    const resolutionValidation = validateCameraResolution(width, height);
    if (!resolutionValidation.valid) {
      newErrors.resolution = resolutionValidation.errors[0];
    }

    const fpsValidation = validateCameraFPS(fps);
    if (!fpsValidation.valid) {
      newErrors.fps = fpsValidation.errors[0];
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSave = async () => {
    if (!ROBOT_ENABLED || saving || !validate()) return;

    setSaving(true);
    try {
      const cameraConfig = {
        backend,
        device,
        width: Number(width),
        height: Number(height),
        fps: Number(fps),
        test_mode: testMode,
      };

      const success = await updateSection(cameraConfig);
      if (success) {
        console.log('Camera configuration saved');
      }
    } catch (error) {
      console.error('Failed to save camera config:', error);
    } finally {
      setSaving(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Camera configuration is disabled.
      </div>
    );
  }

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading camera configuration...</span>
      </div>
    );
  }

  const backendOptions = [
    { value: 'picamera2', label: 'Picamera2 (Raspberry Pi Camera)' },
    { value: 'v4l2', label: 'V4L2 (USB Webcam)' },
    { value: 'auto', label: 'Auto-detect' },
    { value: 'mock', label: 'Mock (Development)' },
  ];

  return (
    <div className="space-y-4">
      {/* Backend Selection */}
      <div>
        <Label className="text-white mb-2 block">Camera Backend</Label>
        <select
          value={backend}
          onChange={(e) => setBackend(e.target.value)}
          className="w-full px-3 py-2 rounded bg-gray-800 border border-gray-700 text-white focus:outline-none focus:ring-2 focus:ring-purple-500"
        >
          {backendOptions.map((option) => (
            <option key={option.value} value={option.value}>
              {option.label}
            </option>
          ))}
        </select>
        {errors.backend && (
          <p className="text-red-400 text-xs mt-1">{errors.backend}</p>
        )}
      </div>

      {/* Device Path */}
      <div>
        <Label htmlFor="camera-device" className="text-gray-300">Device Path</Label>
        <Input
          id="camera-device"
          value={device}
          onChange={(e) => setDevice(e.target.value)}
          className="bg-gray-800 border-gray-700 text-white"
          placeholder="/dev/video0"
        />
      </div>

      {/* Resolution */}
      <div className="grid grid-cols-2 gap-4">
        <div>
          <Label htmlFor="camera-width" className="text-gray-300">Width</Label>
          <Input
            id="camera-width"
            type="number"
            value={width}
            onChange={(e) => setWidth(Number(e.target.value))}
            className="bg-gray-800 border-gray-700 text-white"
            min={160}
            max={3840}
            step={2}
          />
        </div>
        <div>
          <Label htmlFor="camera-height" className="text-gray-300">Height</Label>
          <Input
            id="camera-height"
            type="number"
            value={height}
            onChange={(e) => setHeight(Number(e.target.value))}
            className="bg-gray-800 border-gray-700 text-white"
            min={120}
            max={2160}
            step={2}
          />
        </div>
      </div>
      {errors.resolution && (
        <p className="text-red-400 text-xs">{errors.resolution}</p>
      )}

      {/* FPS */}
      <div>
        <Label htmlFor="camera-fps" className="text-gray-300">Frames Per Second (FPS)</Label>
        <Input
          id="camera-fps"
          type="number"
          value={fps}
          onChange={(e) => setFps(Number(e.target.value))}
          className="bg-gray-800 border-gray-700 text-white"
          min={1}
          max={120}
        />
        {errors.fps && (
          <p className="text-red-400 text-xs mt-1">{errors.fps}</p>
        )}
      </div>

      {/* Test Mode */}
      <div className="flex items-center gap-2">
        <input
          type="checkbox"
          id="camera-test-mode"
          checked={testMode}
          onChange={(e) => setTestMode(e.target.checked)}
          className="w-4 h-4 rounded border-gray-600 bg-gray-700 text-purple-500"
        />
        <Label htmlFor="camera-test-mode" className="text-gray-300">
          Run hardware diagnostics on startup
        </Label>
      </div>

      {/* Save Button */}
      <div className="flex justify-end">
        <Button
          onClick={handleSave}
          disabled={saving}
          className="bg-purple-600 hover:bg-purple-700 text-white"
        >
          {saving ? (
            <>
              <Loader2 className="w-4 h-4 mr-2 animate-spin" />
              Saving...
            </>
          ) : (
            <>
              <Save className="w-4 h-4 mr-2" />
              Save Camera Config
            </>
          )}
        </Button>
      </div>
    </div>
  );
}

