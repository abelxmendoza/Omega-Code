/**
 * Configuration Diff Utilities
 * 
 * Generates diffs between configuration versions.
 */

export interface ConfigDiff {
  section: string;
  key: string;
  oldValue: any;
  newValue: any;
  type: 'added' | 'removed' | 'modified';
}

/**
 * Generate diff between two configurations
 */
export function generateConfigDiff(
  oldConfig: Record<string, any>,
  newConfig: Record<string, any>
): ConfigDiff[] {
  const diffs: ConfigDiff[] = [];

  // Compare all sections
  const allSections = new Set([
    ...Object.keys(oldConfig),
    ...Object.keys(newConfig),
  ]);

  for (const section of allSections) {
    const oldSection = oldConfig[section] || {};
    const newSection = newConfig[section] || {};

    // Compare keys within section
    const allKeys = new Set([
      ...Object.keys(oldSection),
      ...Object.keys(newSection),
    ]);

    for (const key of allKeys) {
      const oldValue = oldSection[key];
      const newValue = newSection[key];

      if (!(key in oldSection)) {
        // Added
        diffs.push({
          section,
          key,
          oldValue: undefined,
          newValue,
          type: 'added',
        });
      } else if (!(key in newSection)) {
        // Removed
        diffs.push({
          section,
          key,
          oldValue,
          newValue: undefined,
          type: 'removed',
        });
      } else if (JSON.stringify(oldValue) !== JSON.stringify(newValue)) {
        // Modified
        diffs.push({
          section,
          key,
          oldValue,
          newValue,
          type: 'modified',
        });
      }
    }
  }

  return diffs;
}

/**
 * Format diff for display
 */
export function formatDiff(diff: ConfigDiff): string {
  const { section, key, oldValue, newValue, type } = diff;

  switch (type) {
    case 'added':
      return `[${section}] ${key}: added = ${JSON.stringify(newValue)}`;
    case 'removed':
      return `[${section}] ${key}: removed (was ${JSON.stringify(oldValue)})`;
    case 'modified':
      return `[${section}] ${key}: ${JSON.stringify(oldValue)} → ${JSON.stringify(newValue)}`;
    default:
      return `[${section}] ${key}: unknown change`;
  }
}

/**
 * Check if configs are equal
 */
export function configsEqual(
  config1: Record<string, any>,
  config2: Record<string, any>
): boolean {
  return JSON.stringify(config1) === JSON.stringify(config2);
}

