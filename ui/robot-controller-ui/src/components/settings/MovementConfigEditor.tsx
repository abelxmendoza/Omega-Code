/**
 * MovementConfigEditor Component
 * 
 * Edits movement configuration (profile, speed limits).
 */

import React, { useState, useEffect } from 'react';
import { Save, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { useConfigSection } from '@/hooks/useConfigSection';
import { validateMovementProfile, validateSpeed } from '@/utils/validators';
import { ROBOT_ENABLED } from '@/utils/env';

export function MovementConfigEditor() {
  const { section, loading, updateSection } = useConfigSection('movement');
  const [saving, setSaving] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});

  const [defaultProfile, setDefaultProfile] = useState('smooth');
  const [maxSpeed, setMaxSpeed] = useState(4095);
  const [minSpeed, setMinSpeed] = useState(0);
  const [servoCenterOnStartup, setServoCenterOnStartup] = useState(true);

  useEffect(() => {
    if (section) {
      setDefaultProfile(section.default_profile || 'smooth');
      setMaxSpeed(section.max_speed ?? 4095);
      setMinSpeed(section.min_speed ?? 0);
      setServoCenterOnStartup(section.servo_center_on_startup ?? true);
    }
  }, [section]);

  const validate = (): boolean => {
    const newErrors: Record<string, string> = {};

    const profileValidation = validateMovementProfile(defaultProfile);
    if (!profileValidation.valid) {
      newErrors.profile = profileValidation.errors[0];
    }

    const maxSpeedValidation = validateSpeed(maxSpeed);
    if (!maxSpeedValidation.valid) {
      newErrors.maxSpeed = maxSpeedValidation.errors[0];
    }

    const minSpeedValidation = validateSpeed(minSpeed);
    if (!minSpeedValidation.valid) {
      newErrors.minSpeed = minSpeedValidation.errors[0];
    }

    if (minSpeed >= maxSpeed) {
      newErrors.speedRange = 'Minimum speed must be less than maximum speed';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSave = async () => {
    if (!ROBOT_ENABLED || saving || !validate()) return;

    setSaving(true);
    try {
      const movementConfig = {
        default_profile: defaultProfile,
        max_speed: Number(maxSpeed),
        min_speed: Number(minSpeed),
        servo_center_on_startup: servoCenterOnStartup,
      };

      const success = await updateSection(movementConfig);
      if (success) {
        console.log('Movement configuration saved');
      }
    } catch (error) {
      console.error('Failed to save movement config:', error);
    } finally {
      setSaving(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Movement configuration is disabled.
      </div>
    );
  }

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading movement configuration...</span>
      </div>
    );
  }

  const profileOptions = [
    { value: 'smooth', label: 'Smooth', description: 'Gentle acceleration, stable control' },
    { value: 'aggressive', label: 'Aggressive', description: 'Fast acceleration, high performance' },
    { value: 'precision', label: 'Precision', description: 'Fine-tuned control, accurate movements' },
  ];

  return (
    <div className="space-y-4">
      {/* Default Profile */}
      <div>
        <Label className="text-white mb-2 block">Default Movement Profile</Label>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
          {profileOptions.map((option) => (
            <button
              key={option.value}
              onClick={() => setDefaultProfile(option.value)}
              className={`
                p-3 rounded-lg border-2 transition-all text-left
                ${defaultProfile === option.value
                  ? 'border-purple-500 bg-purple-500/10'
                  : 'border-gray-700 bg-gray-800 hover:border-gray-600'
                }
              `}
            >
              <div className="font-semibold text-white capitalize mb-1">{option.label}</div>
              <div className="text-xs text-gray-400">{option.description}</div>
            </button>
          ))}
        </div>
        {errors.profile && (
          <p className="text-red-400 text-xs mt-1">{errors.profile}</p>
        )}
      </div>

      {/* Speed Limits */}
      <div className="grid grid-cols-2 gap-4">
        <div>
          <Label htmlFor="movement-min-speed" className="text-gray-300">Minimum Speed</Label>
          <Input
            id="movement-min-speed"
            type="number"
            value={minSpeed}
            onChange={(e) => setMinSpeed(Number(e.target.value))}
            className="bg-gray-800 border-gray-700 text-white"
            min={0}
            max={4095}
          />
          {errors.minSpeed && (
            <p className="text-red-400 text-xs mt-1">{errors.minSpeed}</p>
          )}
        </div>
        <div>
          <Label htmlFor="movement-max-speed" className="text-gray-300">Maximum Speed</Label>
          <Input
            id="movement-max-speed"
            type="number"
            value={maxSpeed}
            onChange={(e) => setMaxSpeed(Number(e.target.value))}
            className="bg-gray-800 border-gray-700 text-white"
            min={0}
            max={4095}
          />
          {errors.maxSpeed && (
            <p className="text-red-400 text-xs mt-1">{errors.maxSpeed}</p>
          )}
        </div>
      </div>
      {errors.speedRange && (
        <p className="text-red-400 text-xs">{errors.speedRange}</p>
      )}

      {/* Servo Center on Startup */}
      <div className="flex items-center gap-2">
        <input
          type="checkbox"
          id="movement-servo-center"
          checked={servoCenterOnStartup}
          onChange={(e) => setServoCenterOnStartup(e.target.checked)}
          className="w-4 h-4 rounded border-gray-600 bg-gray-700 text-purple-500"
        />
        <Label htmlFor="movement-servo-center" className="text-gray-300">
          Center servo on startup
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
              Save Movement Config
            </>
          )}
        </Button>
      </div>
    </div>
  );
}

