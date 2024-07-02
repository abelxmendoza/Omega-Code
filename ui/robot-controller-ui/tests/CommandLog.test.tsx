// tests/CommandLog.test.tsx
import React from 'react';
import { render, act, screen, waitFor } from '@testing-library/react';
import { Provider } from 'react-redux';
import configureStore from 'redux-mock-store';
import CommandLog from '../src/components/CommandLog';
import { CommandLogProvider, useCommandLog } from '../src/components/CommandLogContext';

const mockStore = configureStore([]);

// Mock the useCommandLog hook
jest.mock('../src/components/CommandLogContext', () => ({
  useCommandLog: jest.fn(),
  CommandLogProvider: ({ children }) => <div>{children}</div>,
}));

const AddCommands: React.FC = () => {
  const { addCommand } = useCommandLog();
  const [commandsAdded, setCommandsAdded] = React.useState(false);

  React.useEffect(() => {
    if (!commandsAdded) {
      console.log('AddCommands: useEffect triggered');
      addCommand('move-up');
      console.log('AddCommands: move-up command added');
      addCommand('move-down');
      console.log('AddCommands: move-down command added');
      setCommandsAdded(true);
    }
  }, [addCommand, commandsAdded]);

  return null;
};

describe('CommandLog', () => {
  let store;
  const addCommandMock = jest.fn();
  const commandsMock = ['move-up', 'move-down'];

  beforeAll(() => {
    console.log('Starting CommandLog tests...');
  });

  beforeEach(() => {
    console.log('Setting up for a new test...');
    store = mockStore({
      commands: { commands: [] }
    });

    store.dispatch = jest.fn(store.dispatch);
    (useCommandLog as jest.Mock).mockReturnValue({ addCommand: addCommandMock, commands: commandsMock });

    // Mock WebSocket
    global.WebSocket = jest.fn(() => ({
      send: jest.fn(),
      close: jest.fn(),
      addEventListener: jest.fn((event, handler) => {
        if (event === 'open') {
          handler();
        }
      }),
      removeEventListener: jest.fn(),
    }));
  });

  afterEach(() => {
    console.log('Cleaning up after test...');
    jest.clearAllMocks();
  });

  afterAll(() => {
    console.log('CommandLog tests completed.');
  });

  it('renders command log with provided commands', async () => {
    console.log('Test: renders command log with provided commands - Start');

    await act(async () => {
      render(
        <Provider store={store}>
          <CommandLogProvider>
            <AddCommands />
            <CommandLog />
          </CommandLogProvider>
        </Provider>
      );
    });

    console.log('Test: Waiting for state updates');
    await waitFor(() => {
      expect(screen.getByText('move-up')).toBeInTheDocument();
      console.log('Test: move-up command is in the document');
      expect(screen.getByText('move-down')).toBeInTheDocument();
      console.log('Test: move-down command is in the document');
    });

    console.log('Test: renders command log with provided commands - End');
  }, 5000); // Set timeout to 5 seconds

  it('adds command from WebSocket message', async () => {
    console.log('Test: adds command from WebSocket message - Start');

    await act(async () => {
      render(
        <Provider store={store}>
          <CommandLogProvider>
            <CommandLog />
          </CommandLogProvider>
        </Provider>
      );
    });

    const mockMessageEvent = new MessageEvent('message', {
      data: JSON.stringify({ command: 'move-left' }),
    });

    await act(async () => {
      const messageHandler = global.WebSocket.mock.instances[0].addEventListener.mock.calls.find(call => call[0] === 'message')[1];
      if (messageHandler) {
        messageHandler(mockMessageEvent);
      }
    });

    console.log('Test: Waiting for state updates');
    await waitFor(() => {
      expect(screen.getByText('move-left')).toBeInTheDocument();
      console.log('Test: move-left command is in the document');
    });

    console.log('Test: adds command from WebSocket message - End');
  }, 5000); // Set timeout to 5 seconds
});