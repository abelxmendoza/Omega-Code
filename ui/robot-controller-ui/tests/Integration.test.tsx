// tests/Integration.test.tsx
import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { Provider } from 'react-redux';
import store from '../src/store';
import ControlPanel from '../src/components/ControlPanel';
import CommandLog from '../src/components/CommandLog';

test('Integration: ControlPanel and CommandLog', () => {
  const { getByText } = render(
    <Provider store={store}>
      <div>
        <ControlPanel
          onUp={() => {}}
          onDown={() => {}}
          onLeft={() => {}}
          onRight={() => {}}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
        <CommandLog />
      </div>
    </Provider>
  );

  // Simulate key presses
  fireEvent.keyDown(window, { key: 'w' });
  fireEvent.keyUp(window, { key: 'w' });
  fireEvent.keyDown(window, { key: 's' });
  fireEvent.keyUp(window, { key: 's' });

  // Check if the commands are in the CommandLog
  expect(getByText('move-up')).toBeInTheDocument();
  expect(getByText('move-down')).toBeInTheDocument();
});
