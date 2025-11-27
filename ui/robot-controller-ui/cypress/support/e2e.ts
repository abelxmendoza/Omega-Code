/**
 * Cypress E2E Support File
 * Global setup and configuration for E2E tests
 */

import './commands';

// Mock API endpoints
beforeEach(() => {
  // Intercept system mode API calls
  cy.intercept('GET', '/api/system/mode/list', {
    statusCode: 200,
    body: [
      { id: 0, name: 'CAMERA_ONLY', description: 'Camera Only' },
      { id: 1, name: 'MOTION_DETECTION', description: 'Motion Detection' },
      { id: 2, name: 'TRACKING', description: 'Tracking' },
      { id: 3, name: 'FACE_DETECTION', description: 'Face Detection' },
      { id: 4, name: 'ARUCO_DETECTION', description: 'ArUco Detection' },
      { id: 5, name: 'RECORDING_ONLY', description: 'Recording Only' },
      { id: 6, name: 'ORIN_ENHANCED', description: 'Orin-Enhanced' },
      { id: 7, name: 'ORIN_NAVIGATION', description: 'Orin Navigation' },
    ],
  }).as('getModeList');

  cy.intercept('GET', '/api/system/mode/status', {
    statusCode: 200,
    body: {
      ok: true,
      mode: 0,
      description: 'Camera Only',
      manual_override: false,
      hybrid_mode: 'pi_only',
      orin_available: false,
    },
  }).as('getModeStatus');

  cy.intercept('POST', '/api/system/mode/set', {
    statusCode: 200,
    body: {
      ok: true,
      message: 'System mode set',
      mode: 0,
    },
  }).as('setSystemMode');

  // Intercept latency API calls
  cy.intercept('GET', '/api/video-proxy/latency', {
    statusCode: 200,
    body: {
      ok: true,
      type: 'pi_only',
      latencies_ms: {
        total_processing_ms: 2.5,
        encode_duration_ms: 1.2,
      },
    },
  }).as('getLatency');

  cy.intercept('GET', '/api/video-proxy/latency/hybrid', {
    statusCode: 503,
    body: {
      ok: false,
      error: 'hybrid_system_not_available',
    },
  }).as('getHybridLatency');
});

// Handle uncaught exceptions
Cypress.on('uncaught:exception', (err, runnable) => {
  // Ignore specific errors that don't affect tests
  if (err.message.includes('ResizeObserver loop limit exceeded')) {
    return false;
  }
  if (err.message.includes('Non-Error promise rejection')) {
    return false;
  }
  return true;
});
