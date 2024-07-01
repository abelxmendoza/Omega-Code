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
import { render, fireEvent, screen } from '@testing-library/react';
import Home from '../components/Home'; // Adjust the import according to your file structure

describe('Home Component', () => {
  test('sends INCREASE_SPEED command when "p" key is pressed', () => {
    render(<Home />);
    fireEvent.keyDown(window, { key: 'p' });
    expect(screen.getByText(/INCREASE_SPEED/i)).toBeInTheDocument();
  });

  test('sends DECREASE_SPEED command when "o" key is pressed', () => {
    render(<Home />);
    fireEvent.keyDown(window, { key: 'o' });
    expect(screen.getByText(/DECREASE_SPEED/i)).toBeInTheDocument();
  });

  test('sends CMD_BUZZER command when space bar is pressed', () => {
    render(<Home />);
    fireEvent.keyDown(window, { key: ' ' });
    expect(screen.getByText(/CMD_BUZZER/i)).toBeInTheDocument();
  });
});
