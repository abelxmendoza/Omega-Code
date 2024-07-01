import '@testing-library/jest-dom';
import fetchMock from 'jest-fetch-mock';
fetchMock.enableMocks();

process.on('deprecation', (warning) => {
  if (warning.name === 'DeprecationWarning' && warning.message.includes('punycode')) {
    return;
  }
  console.warn(warning);
});
