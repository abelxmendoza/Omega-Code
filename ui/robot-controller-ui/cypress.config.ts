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
      on('before:browser:launch', (browser = {}, launchOptions) => {
        console.log('Launching browser with the following options:', launchOptions);
        // Modify launchOptions based on the browser
        return launchOptions;
      });
    },
    baseUrl: 'http://localhost:3000',
  },
});
