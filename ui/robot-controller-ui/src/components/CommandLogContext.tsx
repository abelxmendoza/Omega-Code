/*
# File: /src/components/CommandLogContext.tsx
# Summary:
# Wrapper around CommandContext to provide CommandLogProvider and useCommandLog
# for backward compatibility with existing tests.
*/

import React from 'react';
import { CommandProvider, useCommand } from '../context/CommandContext';

// Re-export the provider with the expected name
export const CommandLogProvider = CommandProvider;

// Create a hook that matches the expected interface
export const useCommandLog = () => {
  const context = useCommand();
  
  // Map the CommandContext interface to the expected CommandLogContext interface
  return {
    commands: context.commands,
    addCommand: context.addCommand,
    popCommand: context.popCommand,
    sendCommand: context.sendCommand,
    status: context.status,
    latencyMs: context.latencyMs,
    wsUrl: context.wsUrl,
  };
};
