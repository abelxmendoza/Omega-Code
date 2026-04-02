/*
# File: cypress/e2e/health-monitor.cy.ts
# E2E tests for the centralized health monitoring system.
#
# Tests the end-to-end behavior visible to the user:
#   1. App loads and renders without a crash banner
#   2. Header service status pills appear
#   3. Battery indicator is rendered
#   4. Camera section is present
#   5. Sensor dashboard is present
#   6. Backend offline state is handled gracefully (no JS error, banner shown or
#      component degrades cleanly)
#   7. Vision mode panel renders
#   8. System mode dashboard renders
#
# Note: These tests run against the dev server (baseUrl = http://localhost:3000).
# They use cy.intercept() to mock backend responses, so no Pi hardware is required.
*/

describe('Health Monitor — App Load', () => {
  beforeEach(() => {
    // Mock the health endpoint so systemHealth service sees backend as up
    cy.intercept('GET', '/api/health', { statusCode: 200, body: { ok: true } }).as('healthCheck');
    // Avoid waiting forever for the camera stream
    cy.intercept('GET', '/api/video-proxy*', { statusCode: 503, body: { placeholder: true } }).as('camera');
    // Mock network summary
    cy.intercept('GET', '/api/net/summary', {
      statusCode: 200,
      body: { linkType: 'wifi', ssid: 'TestNet', online: true, ipv4: '192.168.1.100' },
    }).as('netSummary');

    cy.visit('/');
  });

  it('loads without a full-page error', () => {
    // App renders — no error fallback visible
    cy.get('[role="alert"]').should('not.exist');
  });

  it('renders the Robot Controller header', () => {
    cy.contains('Robot Controller').should('be.visible');
  });

  it('renders server status pills in the header', () => {
    cy.contains('Movement').should('exist');
    cy.contains('Video').should('exist');
  });

  it('renders the Battery indicator', () => {
    cy.contains('Battery').should('exist');
  });

  it('renders the camera frame section', () => {
    // CameraFrame renders a container even when stream is unavailable
    cy.contains('Loading camera').should('exist').or(
      cy.get('img[alt="Front Camera"]').should('exist')
    );
  });

  it('renders the Vision Mode panel', () => {
    cy.contains('Vision Mode').should('be.visible');
  });

  it('renders the System Mode Control panel', () => {
    cy.contains('System Mode Control').should('be.visible');
  });

  it('renders the Performance Dashboard section', () => {
    cy.contains('Performance Dashboard').should('be.visible');
  });
});

describe('Health Monitor — Backend Offline Handling', () => {
  beforeEach(() => {
    // Simulate backend down: /api/health returns 503
    cy.intercept('GET', '/api/health', { statusCode: 503 }).as('healthFail');
    cy.intercept('GET', '/api/video-proxy*', { statusCode: 503, body: { placeholder: true } }).as('camera');
    cy.intercept('GET', '/api/net/summary', { statusCode: 503 }).as('netFail');
    cy.intercept('GET', '/api/system/mode/status', { statusCode: 503 }).as('modeFail');

    cy.visit('/');
  });

  it('does not crash — app remains interactive', () => {
    // No full-page error fallback
    cy.get('[role="alert"]').should('not.exist');
    // The main content is still present
    cy.contains('Robot Controller').should('be.visible');
  });

  it('shows "Backend offline" or similar in Performance Dashboard when backend is down', () => {
    // PerformanceDashboard skips polling when health.connected=false
    // It should display a friendly offline message rather than fake data
    cy.contains(/backend offline|waiting for connection|monitoring not available/i).should('exist');
  });

  it('header service pills show disconnected state gracefully', () => {
    // Pills render in error state — no JS exception
    cy.contains('Movement').should('exist');
  });
});

describe('Health Monitor — No ws:// fetch errors', () => {
  it('does not produce ws:// fetch-related console errors', () => {
    // Capture console errors during page load
    const errors: string[] = [];
    cy.on('window:before:load', (win) => {
      cy.stub(win.console, 'error').callsFake((...args: any[]) => {
        errors.push(args.join(' '));
      });
    });

    cy.intercept('GET', '/api/health', { statusCode: 200, body: { ok: true } }).as('health');
    cy.visit('/');
    cy.wait(2000);

    cy.wrap(null).then(() => {
      const wsErrors = errors.filter(e =>
        e.includes('fetch') && (e.includes('ws://') || e.includes('wss://'))
      );
      expect(wsErrors).to.have.length(0);
    });
  });
});
