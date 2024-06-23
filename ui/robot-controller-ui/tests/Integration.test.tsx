import React from 'react';
import { render, fireEvent, screen } from '@testing-library/react';
import { Provider } from 'react-redux';
import store from '../src/redux/store';
import { CommandLogProvider } from '../src/components/CommandLogContext';
import ControlPanel from '../src/components/ControlPanel';
import CommandLog from '../src/components/CommandLog';

test('Integration: ControlPanel and CommandLog', () => {
  render(
    <Provider store={store}>
      <CommandLogProvider>
        <ControlPanel
          onUp={() => {}}
          onDown={() => {}}
          onLeft={() => {}}
          onRight={() => {}}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
        <CommandLog />
      </CommandLogProvider>
    </Provider>
  );

  // Simulate key presses
  fireEvent.keyDown(window, { key: 'w' });
  fireEvent.keyUp(window, { key: 'w' });
  fireEvent.keyDown(window, { key: 's' });
  fireEvent.keyUp(window, { key: 's' });

  // Check if the commands are in the CommandLog
  expect(screen.getByText('move-up')).toBeInTheDocument();
  expect(screen.getByText('move-down')).toBeInTheDocument();
});
