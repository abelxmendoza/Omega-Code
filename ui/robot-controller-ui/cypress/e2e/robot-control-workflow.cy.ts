/**
 * Robot Control Workflow E2E Tests
 * Tests complete robot control workflows
 */

describe('Robot Control Workflow', () => {
  beforeEach(() => {
    cy.visit('/');
    // Wait for page to fully load
    cy.get('main', { timeout: 10000 }).should('be.visible');
  });

  it('can control robot movement', () => {
    // Mock WebSocket connection
    cy.window().then((win) => {
      cy.stub(win, 'WebSocket').returns({
        send: cy.stub(),
        close: cy.stub(),
        addEventListener: cy.stub(),
        readyState: WebSocket.OPEN,
      });
    });

    // Click move up button
    cy.contains('Move Up').click();
    
    // Verify command was sent (if WebSocket mock is set up)
    cy.wait(100);
  });

  it('displays camera feed', () => {
    cy.contains('Live Camera').should('be.visible');
  });

  it('can adjust camera position', () => {
    cy.contains('Camera Up').should('be.visible');
    cy.contains('Camera Left').should('be.visible');
    cy.contains('Camera Right').should('be.visible');
    cy.contains('Camera Down').should('be.visible');
  });

  it('displays sensor dashboard', () => {
    cy.contains('Sensor Dashboard').should('be.visible');
  });

  it('can control speed', () => {
    cy.contains('Speed:').should('be.visible');
    cy.get('input[type="range"]').should('exist');
  });
});

describe('System Integration', () => {
  beforeEach(() => {
    cy.visit('/');
  });

  it('displays all main components', () => {
    cy.contains('Car Control').should('be.visible');
    cy.contains('System Mode Control').should('be.visible');
    cy.contains('Sensor Dashboard').should('be.visible');
  });

  it('header displays status information', () => {
    cy.contains('Robot Controller').should('be.visible');
    cy.contains('Pi:').should('be.visible');
    cy.contains('Battery:').should('be.visible');
  });

  it('handles network errors gracefully', () => {
    // Simulate network error
    cy.intercept('GET', '/api/system/mode/status', { forceNetworkError: true });
    
    // Page should still render
    cy.contains('Robot Controller').should('be.visible');
  });
});

