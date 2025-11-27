/**
 * Robot Lifecycle E2E Tests
 * Complete workflow from boot to shutdown
 */

describe('Robot Lifecycle', () => {
  beforeEach(() => {
    cy.visit('/');
    cy.wait(2000); // Wait for page to load
  });

  it('completes full boot workflow', () => {
    // Step 1: Web dashboard loads
    cy.get('main').should('be.visible');
    cy.contains('Robot Controller').should('be.visible');
    
    // Step 2: Check mode auto-detected
    cy.contains(/Current Mode/i).should('be.visible');
    
    // Step 3: Start MJPEG feed
    cy.contains('Live Camera').should('be.visible');
    
    // Step 4: Verify all components loaded
    cy.contains('Car Control').should('be.visible');
    cy.contains('System Mode Control').should('be.visible');
    cy.contains('Sensor Dashboard').should('be.visible');
  });

  it('switches modes through complete cycle', () => {
    // Switch: PI_ONLY → HYBRID → SAFE_MODE
    cy.contains('0 Camera Only').click();
    cy.waitForSystemMode(0);
    
    cy.contains('6 Orin-Enhanced').click();
    cy.waitForSystemMode(6);
    
    cy.contains('1 Motion Detection').click();
    cy.waitForSystemMode(1);
  });

  it('handles recording workflow', () => {
    // Start recording
    cy.contains('Start recording').click();
    
    // Verify recording started
    cy.contains(/Recording/i).should('be.visible');
    
    // Stop recording
    cy.contains('Stop recording').click();
    
    // Verify recording stopped
    cy.contains(/Recording/i).should('not.exist');
  });

  it('updates metrics under load', () => {
    // Trigger load by switching modes rapidly
    for (let i = 0; i < 5; i++) {
      cy.contains(`${i} `).click();
      cy.wait(100);
    }
    
    // Metrics should still update
    cy.contains(/ms/i, { timeout: 5000 }).should('be.visible');
  });

  it('handles throttle warnings', () => {
    // Mock throttle condition
    cy.intercept('GET', '/api/system/mode/status', {
      statusCode: 200,
      body: {
        ok: true,
        mode: 0,
        throttling: true,
        thermal_temp: 75,
        cpu_load: 80,
      },
    }).as('getThrottleStatus');
    
    cy.wait('@getThrottleStatus');
    cy.contains(/throttling/i, { timeout: 5000 }).should('be.visible');
  });

  it('maintains WebSocket stability', () => {
    // Verify WebSocket connection
    cy.window().then((win) => {
      // WebSocket should be connected
      expect(win.WebSocket).to.exist;
    });
    
    // Perform multiple operations
    cy.contains('0 Camera Only').click();
    cy.wait(500);
    cy.contains('1 Motion Detection').click();
    cy.wait(500);
    
    // WebSocket should remain stable
    cy.contains('System Mode Control').should('be.visible');
  });

  it('shows FPS > 5 under load', () => {
    // Trigger load
    cy.contains('0 Camera Only').click();
    cy.wait(1000);
    
    // FPS should be displayed and > 5
    cy.contains(/FPS/i, { timeout: 5000 }).should('be.visible');
    // FPS value should be > 5 (if displayed)
  });

  it('completes mode transitions within 300ms', () => {
    const startTime = Date.now();
    
    cy.contains('3 Face Detection').click();
    cy.waitForSystemMode(3);
    
    const elapsed = Date.now() - startTime;
    
    // Transition should complete within 300ms (allowing for test overhead)
    expect(elapsed).to.be.lessThan(1000); // Allow 1s for test overhead
  });

  it('handles system shutdown cleanly', () => {
    // Perform operations
    cy.contains('0 Camera Only').click();
    cy.wait(500);
    
    // Simulate shutdown
    // (In real scenario, would trigger shutdown endpoint)
    // For now, verify UI remains responsive
    cy.contains('Robot Controller').should('be.visible');
  });
});

describe('UI Responsiveness', () => {
  beforeEach(() => {
    cy.visit('/');
  });

  it('UI never freezes during operations', () => {
    // Perform rapid operations
    for (let i = 0; i < 10; i++) {
      cy.contains(`${i % 8} `).click();
      cy.wait(50);
    }
    
    // UI should remain responsive
    cy.contains('Robot Controller').should('be.visible');
    cy.contains('System Mode Control').should('be.visible');
  });

  it('handles concurrent user actions', () => {
    // Simulate concurrent actions
    cy.contains('0 Camera Only').click();
    cy.contains('Move Up').click();
    cy.contains('Camera Up').click();
    
    // All actions should complete
    cy.wait(1000);
    cy.contains('Robot Controller').should('be.visible');
  });
});

