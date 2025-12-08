/**
 * LightingConfigEditor Component
 * 
 * Edits lighting configuration (default pattern, brightness).
 */

import React, { useState, useEffect } from 'react';
import { Save, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { useConfigSection } from '@/hooks/useConfigSection';
import { validateBrightness } from '@/utils/validators';
import { ROBOT_ENABLED } from '@/utils/env';

export function LightingConfigEditor() {
  const { section, loading, updateSection } = useConfigSection('lighting');
  const [saving, setSaving] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});

  const [defaultPattern, setDefaultPattern] = useState('omega_signature');
  const [defaultBrightness, setDefaultBrightness] = useState(0.5);
  const [ledCount, setLedCount] = useState(16);
  const [gpioPin, setGpioPin] = useState(18);

  useEffect(() => {
    if (section) {
      setDefaultPattern(section.default_pattern || 'omega_signature');
      setDefaultBrightness(section.default_brightness ?? 0.5);
      setLedCount(section.led_count ?? 16);
      setGpioPin(section.gpio_pin ?? 18);
    }
  }, [section]);

  const validate = (): boolean => {
    const newErrors: Record<string, string> = {};

    const brightnessValidation = validateBrightness(defaultBrightness);
    if (!brightnessValidation.valid) {
      newErrors.brightness = brightnessValidation.errors[0];
    }

    if (ledCount < 1 || ledCount > 1000) {
      newErrors.ledCount = 'LED count must be between 1 and 1000';
    }

    if (gpioPin < 0 || gpioPin > 27) {
      newErrors.gpioPin = 'GPIO pin must be between 0 and 27';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSave = async () => {
    if (!ROBOT_ENABLED || saving || !validate()) return;

    setSaving(true);
    try {
      const lightingConfig = {
        default_pattern: defaultPattern,
        default_brightness: Number(defaultBrightness),
        led_count: Number(ledCount),
        gpio_pin: Number(gpioPin),
      };

      const success = await updateSection(lightingConfig);
      if (success) {
        console.log('Lighting configuration saved');
      }
    } catch (error) {
      console.error('Failed to save lighting config:', error);
    } finally {
      setSaving(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Lighting configuration is disabled.
      </div>
    );
  }

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading lighting configuration...</span>
      </div>
    );
  }

  const patternOptions = [
    'omega_signature',
    'rainbow',
    'aurora',
    'matrix',
    'fire',
    'rave',
    'breathing',
    'static',
    'pulse',
    'chase',
    'theater',
    'wipe',
    'twinkle',
  ];

  return (
    <div className="space-y-4">
      {/* Default Pattern */}
      <div>
        <Label className="text-white mb-2 block">Default Pattern</Label>
        <select
          value={defaultPattern}
          onChange={(e) => setDefaultPattern(e.target.value)}
          className="w-full px-3 py-2 rounded bg-gray-800 border border-gray-700 text-white focus:outline-none focus:ring-2 focus:ring-purple-500"
        >
          {patternOptions.map((pattern) => (
            <option key={pattern} value={pattern}>
              {pattern.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
            </option>
          ))}
        </select>
      </div>

      {/* Brightness */}
      <div>
        <Label htmlFor="lighting-brightness" className="text-gray-300">
          Default Brightness: {(defaultBrightness * 100).toFixed(0)}%
        </Label>
        <input
          id="lighting-brightness"
          type="range"
          min="0"
          max="1"
          step="0.01"
          value={defaultBrightness}
          onChange={(e) => setDefaultBrightness(Number(e.target.value))}
          className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-purple-500"
        />
        {errors.brightness && (
          <p className="text-red-400 text-xs mt-1">{errors.brightness}</p>
        )}
      </div>

      {/* LED Count */}
      <div>
        <Label htmlFor="lighting-led-count" className="text-gray-300">LED Count</Label>
        <Input
          id="lighting-led-count"
          type="number"
          value={ledCount}
          onChange={(e) => setLedCount(Number(e.target.value))}
          className="bg-gray-800 border-gray-700 text-white"
          min={1}
          max={1000}
        />
        {errors.ledCount && (
          <p className="text-red-400 text-xs mt-1">{errors.ledCount}</p>
        )}
      </div>

      {/* GPIO Pin */}
      <div>
        <Label htmlFor="lighting-gpio-pin" className="text-gray-300">GPIO Pin</Label>
        <Input
          id="lighting-gpio-pin"
          type="number"
          value={gpioPin}
          onChange={(e) => setGpioPin(Number(e.target.value))}
          className="bg-gray-800 border-gray-700 text-white"
          min={0}
          max={27}
        />
        {errors.gpioPin && (
          <p className="text-red-400 text-xs mt-1">{errors.gpioPin}</p>
        )}
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
              Save Lighting Config
            </>
          )}
        </Button>
      </div>
    </div>
  );
}

