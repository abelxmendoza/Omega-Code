/**
 * Cypress Custom Commands
 * Reusable commands for E2E tests
 */

/// <reference types="cypress" />

declare global {
  namespace Cypress {
    interface Chainable {
      /**
       * Wait for system mode to be set
       */
      waitForSystemMode(mode: number): Chainable<void>;
      
      /**
       * Mock WebSocket connection
       */
      mockWebSocket(): Chainable<void>;
      
      /**
       * Wait for latency metrics to load
       */
      waitForLatencyMetrics(): Chainable<void>;
      
      /**
       * Switch system mode
       */
      switchSystemMode(mode: number): Chainable<void>;
    }
  }
}

Cypress.Commands.add('waitForSystemMode', (mode: number) => {
  cy.contains(new RegExp(`Mode ${mode}:`, 'i')).should('be.visible');
});

Cypress.Commands.add('mockWebSocket', () => {
  cy.window().then((win) => {
    cy.stub(win, 'WebSocket').returns({
      send: cy.stub(),
      close: cy.stub(),
      addEventListener: cy.stub(),
      removeEventListener: cy.stub(),
      readyState: WebSocket.OPEN,
      CONNECTING: WebSocket.CONNECTING,
      OPEN: WebSocket.OPEN,
      CLOSING: WebSocket.CLOSING,
      CLOSED: WebSocket.CLOSED,
    });
  });
});

Cypress.Commands.add('waitForLatencyMetrics', () => {
  cy.contains(/Latency Metrics/i).should('be.visible');
  cy.contains(/ms/i, { timeout: 5000 }).should('be.visible');
});

Cypress.Commands.add('switchSystemMode', (mode: number) => {
  cy.contains(new RegExp(`^${mode} `)).click();
  cy.wait('@setSystemMode');
  cy.waitForSystemMode(mode);
});

export {};
