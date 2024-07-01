// tests/CommandLog.test.tsx
import React from 'react';
import { render, act, screen } from '@testing-library/react';
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
  });

  afterEach(() => {
    console.log('Cleaning up after test...');
    jest.clearAllMocks();
  });

  afterAll(() => {
    console.log('CommandLog tests completed.');
  });

  it.only('renders command log with provided commands', async () => {
    console.log('Test: renders command log with provided commands - Start');

    try {
      await act(async () => {
        render(
          <Provider store={store}>
            <CommandLogProvider>
              <AddCommands />
              <CommandLog />
            </CommandLogProvider>
          </Provider>
        );

        console.log('Test: Waiting for state updates');
        await new Promise((resolve) => setTimeout(resolve, 1000));
      });

      console.log('Test: Checking if commands are rendered');
      expect(screen.getByText('move-up')).toBeInTheDocument();
      console.log('Test: move-up command is in the document');
      expect(screen.getByText('move-down')).toBeInTheDocument();
      console.log('Test: move-down command is in the document');
    } catch (error) {
      console.error('Test: Error in renders command log with provided commands', error);
      throw error;
    }
    console.log('Test: renders command log with provided commands - End');
  }, 5000); // Set timeout to 5 seconds
});
