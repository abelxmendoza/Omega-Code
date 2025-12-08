/**
 * SettingsSection Component
 * 
 * Reusable wrapper for settings sections with expand/collapse.
 */

import React, { useState, ReactNode } from 'react';
import { ChevronDown, ChevronUp } from 'lucide-react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';

interface SettingsSectionProps {
  title: string;
  description?: string;
  defaultExpanded?: boolean;
  children: ReactNode;
  className?: string;
}

export function SettingsSection({
  title,
  description,
  defaultExpanded = true,
  children,
  className = '',
}: SettingsSectionProps) {
  const [expanded, setExpanded] = useState(defaultExpanded);

  return (
    <Card className={`bg-gray-800 border-gray-700 ${className}`}>
      <CardHeader
        className="cursor-pointer select-none hover:bg-gray-750 transition-colors"
        onClick={() => setExpanded(!expanded)}
      >
        <div className="flex items-center justify-between">
          <div className="flex-1">
            <CardTitle className="text-lg text-white">{title}</CardTitle>
            {description && (
              <p className="text-sm text-gray-400 mt-1">{description}</p>
            )}
          </div>
          <div className="ml-4">
            {expanded ? (
              <ChevronUp className="w-5 h-5 text-gray-400" />
            ) : (
              <ChevronDown className="w-5 h-5 text-gray-400" />
            )}
          </div>
        </div>
      </CardHeader>
      {expanded && (
        <CardContent className="pt-4">
          {children}
        </CardContent>
      )}
    </Card>
  );
}

