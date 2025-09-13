/*
# File: /Omega-Code/ui/robot-controller-ui/tests/setupTests.ts
# Summary:
#   Jest test setup. Extends RTL matchers and provides a minimal global fetch mock
#   so tests that don’t stub fetch themselves still run. Uses @jest/globals so
#   “Cannot use namespace 'jest' as a value” never occurs during type-check.
*/

import '@testing-library/jest-dom';
import { jest } from '@jest/globals';

/** Minimal fetch mock used by tests that don't stub fetch explicitly. */
const mockFetch = jest.fn(() =>
  Promise.resolve({
    ok: true,
    json: () => Promise.resolve({}),
  } as any)
) as unknown as typeof fetch;

// Install on the global scope used by both Node and JSDOM.
globalThis.fetch = mockFetch;

export {};
