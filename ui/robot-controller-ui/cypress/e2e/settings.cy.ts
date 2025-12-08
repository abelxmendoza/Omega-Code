/**
 * End-to-End Tests for Settings Page
 * 
 * Tests complete user workflows for robot configuration.
 */

describe('Settings Page E2E', () => {
  beforeEach(() => {
    // Mock API responses
    cy.intercept('GET', '**/api/config', {
      statusCode: 200,
      body: {
        ok: true,
        config: {
          robot: {
            name: 'Omega-1',
            profile: 'pi4b',
            version: '1.0.0',
          },
          network: {
            default_mode: 'ap',
            ap: {
              ssid: 'Omega1-AP',
              password: 'omegawifi123',
              ip: '192.168.4.1',
            },
          },
          camera: {
            backend: 'picamera2',
            width: 640,
            height: 480,
            fps: 30,
          },
          movement: {
            default_profile: 'smooth',
            max_speed: 4095,
            min_speed: 0,
          },
          lighting: {
            default_pattern: 'omega_signature',
            default_brightness: 0.5,
          },
        },
      },
    }).as('getConfig');

    cy.intercept('POST', '**/api/config/robot', {
      statusCode: 200,
      body: { ok: true, data: { name: 'Omega-1-Updated' } },
    }).as('updateRobotConfig');

    cy.intercept('GET', '**/api/config/profile/*', {
      statusCode: 200,
      body: {
        ok: true,
        data: {
          hardware: { cpu_cores: 4, ram_gb: 4 },
          capabilities: { ml_capable: false },
        },
      },
    }).as('getProfile');
  });

  it('loads settings page', () => {
    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.contains('Robot Settings').should('be.visible');
    cy.contains('Network Settings').should('be.visible');
    cy.contains('Camera Settings').should('be.visible');
  });

  it('displays robot name', () => {
    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.get('input[placeholder="Omega-1"]').should('have.value', 'Omega-1');
  });

  it('updates robot name', () => {
    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.get('input[placeholder="Omega-1"]')
      .clear()
      .type('Omega-1-Updated')
      .blur();

    cy.wait('@updateRobotConfig');
    cy.contains('Omega-1-Updated').should('exist');
  });

  it('switches robot profile', () => {
    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.contains('jetson').click();
    cy.wait('@getProfile');

    cy.contains('jetson').should('have.class', 'border-purple-500');
  });

  it('expands and collapses settings sections', () => {
    cy.visit('/settings');
    cy.wait('@getConfig');

    // Network section should be expanded by default
    cy.contains('Network Settings').click();
    
    // Click again to collapse
    cy.contains('Network Settings').click();
  });

  it('configures network settings', () => {
    cy.intercept('POST', '**/api/config/network', {
      statusCode: 200,
      body: { ok: true },
    }).as('updateNetworkConfig');

    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.contains('Network Settings').click();
    
    // Change SSID
    cy.get('input[id="ap-ssid"]').clear().type('NewSSID');
    
    // Save
    cy.contains('Save Network Config').click();
    cy.wait('@updateNetworkConfig');
  });

  it('configures camera settings', () => {
    cy.intercept('POST', '**/api/config/camera', {
      statusCode: 200,
      body: { ok: true },
    }).as('updateCameraConfig');

    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.contains('Camera Settings').click();
    
    // Change resolution
    cy.get('input[id="camera-width"]').clear().type('1280');
    cy.get('input[id="camera-height"]').clear().type('720');
    
    // Save
    cy.contains('Save Camera Config').click();
    cy.wait('@updateCameraConfig');
  });

  it('validates configuration', () => {
    cy.intercept('GET', '**/api/config/validate', {
      statusCode: 200,
      body: {
        ok: true,
        valid: true,
        errors: [],
      },
    }).as('validateConfig');

    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.contains('Apply Changes').click();
    cy.contains('Validate Config').click();
    cy.wait('@validateConfig');

    cy.contains('Configuration Valid').should('be.visible');
  });

  it('exports configuration', () => {
    cy.intercept('GET', '**/api/config/export', {
      statusCode: 200,
      body: {
        ok: true,
        exported_at: new Date().toISOString(),
      },
    }).as('exportConfig');

    cy.visit('/settings');
    cy.wait('@getConfig');

    cy.contains('Configuration Import/Export').click();
    cy.contains('Export Config').click();
    cy.wait('@exportConfig');
  });

  it('handles offline mode gracefully', () => {
    cy.intercept('GET', '**/api/config', {
      statusCode: 503,
      body: { offline: true },
    }).as('getConfigOffline');

    cy.visit('/settings');
    cy.wait('@getConfigOffline');

    cy.contains(/Robot is offline/i).should('be.visible');
  });
});

