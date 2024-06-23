echo "// Example end-to-end test for the robot controller UI
describe('Robot Controller UI E2E Tests', () => {
  it('should navigate to the control panel and send commands', () => {
    cy.visit('http://localhost:3000');
    cy.contains('Control Panel').click();
    cy.get('button').contains('Up').click();
    cy.get('button').contains('Down').click();
    cy.get('button').contains('Left').click();
    cy.get('button').contains('Right').click();
    cy.get('li').should('contain', 'move-up');
    cy.get('li').should('contain', 'move-down');
    cy.get('li').should('contain', 'move-left');
    cy.get('li').should('contain', 'move-right');
  });
});" > cypress/integration/e2e_spec.js



// File: /cypress/integration/e2e_spec.js

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
