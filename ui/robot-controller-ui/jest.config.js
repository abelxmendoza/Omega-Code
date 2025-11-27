// jest.config.js
module.exports = {
  testEnvironment: 'jest-environment-jsdom',

  // ✅ runs BEFORE anything else (must include your tests/setupPolyfills.ts)
  setupFiles: ['<rootDir>/tests/setupPolyfills.ts'],

  // ✅ runs after env is ready (your MSW wiring, shims, etc.)
  setupFilesAfterEnv: ['<rootDir>/tests/setupTests.ts'],

  moduleNameMapper: {
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
    '\\.(png|jpe?g|gif|webp|svg)$': '<rootDir>/tests/__mocks__/fileMock.js',
    '^@/(.*)$': '<rootDir>/src/$1',
    '^leaflet$': '<rootDir>/tests/__mocks__/leaflet.js',
  },

  transform: {
    '^.+\\.(js|jsx|ts|tsx)$': 'babel-jest',
  },

  transformIgnorePatterns: [
    '/node_modules/(?!react-leaflet|@react-leaflet|leaflet|msw|@mswjs|until-async)',
  ],

  testPathIgnorePatterns: [
    '<rootDir>/.next/',
    '<rootDir>/node_modules/',
    '<rootDir>/cypress/',
  ],

  moduleFileExtensions: ['js', 'jsx', 'ts', 'tsx', 'json', 'node'],
  
  // Coverage thresholds
  coverageThreshold: {
    global: {
      statements: 80,
      branches: 75,
      lines: 80,
      functions: 75,
    },
  },
};
