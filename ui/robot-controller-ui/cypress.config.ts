// cypress.config.ts
import { defineConfig } from 'cypress';

export default defineConfig({
  projectId: 'muvqwy',
  e2e: {
    setupNodeEvents(on, config) {
      // Example: Implement a task to reset the database
      on('task', {
        resetDatabase() {
          // Implement the logic to reset the database
          console.log('Resetting the database...');
          return null;
        },
        // Other custom tasks can be added here
      });

      // Example: Implement an environment variable setup
      on('before:browser:launch', (browser, launchOptions) => {
        if (browser) {
          console.log(`Launching browser: ${browser.name}`);
          console.log('With the following options:', launchOptions);
          // Modify launchOptions based on the browser type
        } else {
          console.warn('Browser information is missing.');
        }
        return launchOptions;
      });

      // Return the modified config if needed
      return config;
    },
    baseUrl: 'http://localhost:3000',
  },
});
