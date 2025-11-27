/**
 * System Mode Workflow E2E Tests
 * 
 * Tests the complete system mode switching workflow
 */

describe('System Mode Workflow', () => {
  beforeEach(() => {
    cy.visit('/');
    // Wait for page to load
    cy.get('[data-testid="system-mode-dashboard"]', { timeout: 10000 }).should('be.visible');
  });

  it('displays all system mode buttons', () => {
    cy.contains('System Mode Control').should('be.visible');
    cy.contains('0 Camera Only').should('be.visible');
    cy.contains('1 Motion Detection').should('be.visible');
    cy.contains('2 Tracking').should('be.visible');
    cy.contains('3 Face Detection').should('be.visible');
    cy.contains('4 ArUco Detection').should('be.visible');
    cy.contains('5 Recording Only').should('be.visible');
    cy.contains('6 Orin-Enhanced').should('be.visible');
    cy.contains('7 Orin Navigation').should('be.visible');
  });

  it('switches to motion detection mode', () => {
    cy.contains('1 Motion Detection').click();
    
    // Wait for API call
    cy.wait('@setSystemMode').then((interception) => {
      expect(interception.request.body).to.deep.equal({ mode: 1 });
      expect(interception.response?.statusCode).to.eq(200);
    });

    // Verify mode change
    cy.contains(/Mode 1: Motion Detection/i).should('be.visible');
  });

  it('displays current mode status', () => {
    cy.contains(/Current Mode/i).should('be.visible');
    cy.contains(/Mode \d+:/i).should('be.visible');
  });

  it('shows thermal warnings when throttling', () => {
    // Mock throttling status
    cy.intercept('GET', '/api/system/mode/status', {
      statusCode: 200,
      body: {
        ok: true,
        mode: 0,
        throttling: true,
        thermal_temp: 75,
        cpu_load: 80,
      },
    }).as('getStatus');

    cy.wait('@getStatus');
    cy.contains(/throttling/i).should('be.visible');
  });

  it('handles mode switching errors gracefully', () => {
    cy.intercept('POST', '/api/system/mode/set', {
      statusCode: 500,
      body: { error: 'Failed to set mode' },
    }).as('setModeError');

    cy.contains('3 Face Detection').click();
    cy.wait('@setModeError');
    
    // Error should be displayed but page should remain functional
    cy.contains(/System Mode Control/i).should('be.visible');
  });
});

describe('Latency Metrics Display', () => {
  beforeEach(() => {
    cy.visit('/');
  });

  it('displays latency metrics in header', () => {
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

    cy.wait('@getLatency');
    cy.contains(/Pi:/i).should('be.visible');
    cy.contains(/ms/i).should('be.visible');
  });

  it('displays hybrid latency when Orin is connected', () => {
    cy.intercept('GET', '/api/video-proxy/latency/hybrid', {
      statusCode: 200,
      body: {
        ok: true,
        type: 'hybrid',
        round_trip_ms: { avg: 45.2, min: 40.0, max: 50.0, count: 100 },
        inference_ms: { avg: 12.3, min: 10.0, max: 15.0, count: 100 },
      },
    }).as('getHybridLatency');

    cy.wait('@getHybridLatency');
    cy.contains(/Hybrid:/i).should('be.visible');
    cy.contains(/45.2.*ms/i).should('be.visible');
  });
});

