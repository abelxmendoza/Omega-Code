// File: /cypress/e2e/e2e_spec.ts

describe('ControlPanel E2E Test', () => {
    beforeEach(() => {
      cy.visit('/');
    });
  
    it('should render ControlPanel with directional buttons and respond to clicks', () => {
      // Check if buttons are rendered
      cy.contains('Up').should('exist');
      cy.contains('Down').should('exist');
      cy.contains('Left').should('exist');
      cy.contains('Right').should('exist');
  
      // Simulate button clicks
      cy.contains('Up').click();
      cy.contains('Down').click();
      cy.contains('Left').click();
      cy.contains('Right').click();
  
      // Check if the log contains the commands
      cy.get('ul').should('contain', 'move-up');
      cy.get('ul').should('contain', 'move-down');
      cy.get('ul').should('contain', 'move-left');
      cy.get('ul').should('contain', 'move-right');
    });
  });
  