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
import { useCommandLog } from '../src/components/CommandLogContext';

// Mock the useCommandLog hook
jest.mock('../src/components/CommandLogContext', () => ({
  useCommandLog: jest.fn(),
  CommandLogProvider: ({ children }) => <div>{children}</div>,
}));

describe('Home Component', () => {
  const addCommandMock = jest.fn();

  beforeEach(() => {
    (useCommandLog as jest.Mock).mockReturnValue({ addCommand: addCommandMock });
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  test('sends INCREASE_SPEED command when "p" key is pressed', async () => {
    await act(async () => {
      render(<Home />);
    });
    fireEvent.keyDown(window, { key: 'p' });
    expect(addCommandMock).toHaveBeenCalledWith(expect.stringContaining(COMMAND.INCREASE_SPEED));
  });

  test('sends DECREASE_SPEED command when "o" key is pressed', async () => {
    await act(async () => {
      render(<Home />);
    });
    fireEvent.keyDown(window, { key: 'o' });
    expect(addCommandMock).toHaveBeenCalledWith(expect.stringContaining(COMMAND.DECREASE_SPEED));
  });

  test('sends CMD_BUZZER command when space bar is pressed', async () => {
    await act(async () => {
      render(<Home />);
    });
    fireEvent.keyDown(window, { key: ' ' });
    expect(addCommandMock).toHaveBeenCalledWith(expect.stringContaining(COMMAND.CMD_BUZZER));
  });
});
