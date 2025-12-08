/**
 * ConfigImportExport Component
 * 
 * Handles configuration import/export with diff preview.
 */

import React, { useState, useRef } from 'react';
import { Download, Upload, FileText, AlertCircle, CheckCircle, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';
import { generateConfigDiff, formatDiff, ConfigDiff } from '@/utils/configDiff';
import { useConfig } from '@/hooks/useConfig';

export function ConfigImportExport() {
  const { config, refresh } = useConfig();
  const [importedConfig, setImportedConfig] = useState<any>(null);
  const [diff, setDiff] = useState<ConfigDiff[]>([]);
  const [importing, setImporting] = useState(false);
  const [exporting, setExporting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const handleExport = async () => {
    if (!ROBOT_ENABLED) return;

    setExporting(true);
    setError(null);

    try {
      const response = await robotFetch('/api/config/export');
      
      if (response.offline) {
        setError('Robot is offline');
        return;
      }

      const data = await response.json();
      
      if (data.ok) {
        // Download as JSON file
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `omega-config-${new Date().toISOString().split('T')[0]}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      } else {
        setError('Failed to export configuration');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Export failed');
      console.error('Failed to export config:', err);
    } finally {
      setExporting(false);
    }
  };

  const handleFileSelect = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const imported = JSON.parse(e.target?.result as string);
        setImportedConfig(imported);
        setError(null);

        // Generate diff if we have current config
        if (config && imported.config) {
          const diffResult = generateConfigDiff(config, imported.config);
          setDiff(diffResult);
        }
      } catch (err) {
        setError('Invalid JSON file');
        setImportedConfig(null);
        setDiff([]);
      }
    };
    reader.readAsText(file);
  };

  const handleImport = async () => {
    if (!ROBOT_ENABLED || !importedConfig || importing) return;

    setImporting(true);
    setError(null);

    try {
      const response = await robotFetch('/api/config/import', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(importedConfig),
      });
      
      if (response.offline) {
        setError('Robot is offline');
        return;
      }

      const result = await response.json();
      
      if (result.ok) {
        // Clear imported config and refresh
        setImportedConfig(null);
        setDiff([]);
        if (fileInputRef.current) {
          fileInputRef.current.value = '';
        }
        await refresh();
        console.log('Configuration imported successfully');
      } else {
        setError(result.errors?.join(', ') || 'Import failed');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Import failed');
      console.error('Failed to import config:', err);
    } finally {
      setImporting(false);
    }
  };

  const handleCancelImport = () => {
    setImportedConfig(null);
    setDiff([]);
    setError(null);
    if (fileInputRef.current) {
      fileInputRef.current.value = '';
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Import/export is disabled.
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {/* Export Section */}
      <div className="p-4 bg-gray-900/50 rounded border border-gray-700">
        <h4 className="text-sm font-semibold text-white mb-2">Export Configuration</h4>
        <p className="text-xs text-gray-400 mb-3">
          Download current configuration as JSON file for backup or transfer.
        </p>
        <Button
          onClick={handleExport}
          disabled={exporting}
          variant="outline"
          className="border-gray-600 text-gray-300 hover:bg-gray-700"
        >
          {exporting ? (
            <>
              <Loader2 className="w-4 h-4 mr-2 animate-spin" />
              Exporting...
            </>
          ) : (
            <>
              <Download className="w-4 h-4 mr-2" />
              Export Config
            </>
          )}
        </Button>
      </div>

      {/* Import Section */}
      <div className="p-4 bg-gray-900/50 rounded border border-gray-700">
        <h4 className="text-sm font-semibold text-white mb-2">Import Configuration</h4>
        <p className="text-xs text-gray-400 mb-3">
          Upload a previously exported configuration file. Changes will be previewed before applying.
        </p>
        
        <div className="flex gap-2 mb-3">
          <input
            ref={fileInputRef}
            type="file"
            accept=".json"
            onChange={handleFileSelect}
            className="hidden"
            id="config-file-input"
          />
          <label
            htmlFor="config-file-input"
            className="px-4 py-2 rounded border border-gray-600 bg-gray-800 text-gray-300 hover:bg-gray-700 cursor-pointer inline-flex items-center gap-2"
          >
            <Upload className="w-4 h-4" />
            Select File
          </label>
        </div>

        {error && (
          <div className="p-3 bg-red-900/20 border border-red-500/50 rounded flex items-start gap-2 mb-3">
            <AlertCircle className="w-5 h-5 text-red-400 flex-shrink-0 mt-0.5" />
            <div className="text-sm text-red-400">{error}</div>
          </div>
        )}

        {importedConfig && (
          <div className="space-y-3">
            <div className="p-3 bg-blue-900/20 border border-blue-500/50 rounded flex items-start gap-2">
              <FileText className="w-5 h-5 text-blue-400 flex-shrink-0 mt-0.5" />
              <div className="flex-1">
                <div className="text-sm text-blue-300 font-semibold mb-1">Configuration File Loaded</div>
                {importedConfig.exported_at && (
                  <div className="text-xs text-blue-400">
                    Exported: {new Date(importedConfig.exported_at).toLocaleString()}
                  </div>
                )}
              </div>
            </div>

            {/* Diff Preview */}
            {diff.length > 0 && (
              <div className="p-3 bg-gray-900 rounded border border-gray-700">
                <div className="text-sm font-semibold text-white mb-2">Changes Preview</div>
                <div className="space-y-1 max-h-48 overflow-y-auto">
                  {diff.map((d, i) => (
                    <div
                      key={i}
                      className={`text-xs p-2 rounded ${
                        d.type === 'added'
                          ? 'bg-green-900/20 text-green-300'
                          : d.type === 'removed'
                          ? 'bg-red-900/20 text-red-300'
                          : 'bg-yellow-900/20 text-yellow-300'
                      }`}
                    >
                      {formatDiff(d)}
                    </div>
                  ))}
                </div>
              </div>
            )}

            {/* Import Actions */}
            <div className="flex gap-2">
              <Button
                onClick={handleImport}
                disabled={importing}
                className="bg-green-600 hover:bg-green-700 text-white"
              >
                {importing ? (
                  <>
                    <Loader2 className="w-4 h-4 mr-2 animate-spin" />
                    Importing...
                  </>
                ) : (
                  <>
                    <CheckCircle className="w-4 h-4 mr-2" />
                    Apply Import
                  </>
                )}
              </Button>
              <Button
                onClick={handleCancelImport}
                disabled={importing}
                variant="outline"
                className="border-gray-600 text-gray-300 hover:bg-gray-700"
              >
                Cancel
              </Button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

