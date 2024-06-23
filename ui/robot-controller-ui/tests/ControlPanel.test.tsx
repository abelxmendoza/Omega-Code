// tests/ControlPanel.test.tsx
import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { Provider } from 'react-redux';
import configureStore from 'redux-mock-store';
import ControlPanel from '../src/components/ControlPanel';
import { executeCommand } from '../src/redux/reducers/controlPanelReducer';

const mockStore = configureStore([]);

describe('ControlPanel', () => {
  let store;
  let onUp, onDown, onLeft, onRight;

  beforeEach(() => {
    store = mockStore({ commands: { commands: [] } });
    onUp = jest.fn();
    onDown = jest.fn();
    onLeft = jest.fn();
    onRight = jest.fn();
  });

  it('renders ControlPanel with directional buttons', () => {
    const { getByText } = render(
      <Provider store={store}>
        <ControlPanel
          onUp={onUp}
          onDown={onDown}
          onLeft={onLeft}
          onRight={onRight}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
      </Provider>
    );

    expect(getByText('Up')).toBeInTheDocument();
    expect(getByText('Down')).toBeInTheDocument();
    expect(getByText('Left')).toBeInTheDocument();
    expect(getByText('Right')).toBeInTheDocument();
  });

  it('calls correct function on button click', () => {
    const { getByText } = render(
      <Provider store={store}>
        <ControlPanel
          onUp={onUp}
          onDown={onDown}
          onLeft={onLeft}
          onRight={onRight}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
      </Provider>
    );

    fireEvent.click(getByText('Up'));
    expect(onUp).toHaveBeenCalled();
    fireEvent.click(getByText('Down'));
    expect(onDown).toHaveBeenCalled();
    fireEvent.click(getByText('Left'));
    expect(onLeft).toHaveBeenCalled();
    fireEvent.click(getByText('Right'));
    expect(onRight).toHaveBeenCalled();
  });

  it('calls correct function on key press', () => {
    render(
      <Provider store={store}>
        <ControlPanel
          onUp={onUp}
          onDown={onDown}
          onLeft={onLeft}
          onRight={onRight}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'w' });
    expect(onUp).toHaveBeenCalled();

    fireEvent.keyDown(window, { key: 's' });
    expect(onDown).toHaveBeenCalled();

    fireEvent.keyDown(window, { key: 'a' });
    expect(onLeft).toHaveBeenCalled();

    fireEvent.keyDown(window, { key: 'd' });
    expect(onRight).toHaveBeenCalled();
  });

  it('cleans up event listeners on unmount', () => {
    const { unmount } = render(
      <Provider store={store}>
        <ControlPanel
          onUp={onUp}
          onDown={onDown}
          onLeft={onLeft}
          onRight={onRight}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'w' });
    expect(onUp).toHaveBeenCalledTimes(1);

    unmount();

    onUp.mockClear();
    fireEvent.keyDown(window, { key: 'w' });
    expect(onUp).not.toHaveBeenCalled();
  });

  it('handles different control types', () => {
    const { rerender } = render(
      <Provider store={store}>
        <ControlPanel
          onUp={onUp}
          onDown={onDown}
          onLeft={onLeft}
          onRight={onRight}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="wasd"
        />
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'w' });
    expect(onUp).toHaveBeenCalled();

    rerender(
      <Provider store={store}>
        <ControlPanel
          onUp={onUp}
          onDown={onDown}
          onLeft={onLeft}
          onRight={onRight}
          labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
          controlType="arrows"
        />
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'ArrowUp' });
    expect(onUp).toHaveBeenCalled();
  });
});
