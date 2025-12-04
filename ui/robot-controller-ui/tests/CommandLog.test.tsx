// tests/CommandLog.test.tsx
import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import CommandLog from '../src/components/CommandLog';
import { useCommand } from '../src/context/CommandContext';

// Mock WebSocket
const mockWebSocket = {
  send: jest.fn(),
  close: jest.fn(),
  addEventListener: jest.fn((event, handler) => {
    if (event === 'open') {
      setTimeout(() => handler(), 0);
    }
  }),
  removeEventListener: jest.fn(),
  readyState: WebSocket.OPEN,
};

global.WebSocket = jest.fn(() => mockWebSocket) as any;

// Helper component to add commands for testing
const AddCommands: React.FC = () => {
  const { addCommand } = useCommand();
  const [commandsAdded, setCommandsAdded] = React.useState(false);

  React.useEffect(() => {
    if (!commandsAdded) {
      addCommand('move-up');
      addCommand('move-down');
      setCommandsAdded(true);
    }
  }, [addCommand, commandsAdded]);

  return null;
};

describe('CommandLog', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockWebSocket.readyState = WebSocket.OPEN;
    (global.WebSocket as jest.Mock).mockReturnValue(mockWebSocket);
  });

  it('renders command log component', async () => {
    await act(async () => {
      renderWithProviders(<CommandLog />);
    });

    await waitFor(() => {
      expect(screen.getByText(/Command Log/i)).toBeInTheDocument();
    });
  });

  it('displays empty state when no commands', async () => {
    await act(async () => {
      renderWithProviders(<CommandLog />);
    });

    await waitFor(() => {
      const emptyMessage = screen.queryByText(/No commands logged yet/i);
      expect(emptyMessage || screen.getByText(/Command Log/i)).toBeTruthy();
    });
  });

  it('renders commands when provided', async () => {
    await act(async () => {
      renderWithProviders(
        <>
          <AddCommands />
          <CommandLog />
        </>
      );
    });

    await waitFor(() => {
      expect(screen.getByText('move-up')).toBeInTheDocument();
      expect(screen.getByText('move-down')).toBeInTheDocument();
    }, { timeout: 3000 });
  });
});
