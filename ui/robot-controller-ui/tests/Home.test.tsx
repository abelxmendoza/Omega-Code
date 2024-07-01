/**
 * File Path: src/pages/index.tsx
 * 
 * Home Component
 * 
 * This component is the main entry point for the robot controller application.
 * It provides the UI for controlling the robot, viewing sensor data, and managing command logs.
 * The component handles sending commands to the robot and integrates multiple sub-components.
 */

import React from 'react';
import { render, fireEvent, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import Home from '../src/pages/index'; // Adjust the import according to your file structure
import { COMMAND } from '../src/control_definitions';
import { CommandLogProvider } from '../src/components/CommandLogContext'; // Import the provider

describe('Home Component', () => {
  const renderWithProvider = (ui) => {
    return render(
      <CommandLogProvider>
        {ui}
      </CommandLogProvider>
    );
  };

  test('renders Home component', () => {
    const { getByText } = renderWithProvider(<Home />);
    expect(getByText('Robot Controller')).toBeInTheDocument();
  });

  test('sends INCREASE_SPEED command when "p" key is pressed', () => {
    const { container } = renderWithProvider(<Home />);
    fireEvent.keyDown(container, { key: 'p' });
    // Add your assertion here
  });
});
