import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { Provider } from 'react-redux';
import store from '../src/redux/store';
import { CommandLogProvider } from '../src/components/CommandLogContext';
import ControlPanel from '../src/components/ControlPanel';

describe('ControlPanel', () => {
  it('renders ControlPanel with directional buttons', () => {
    const { getByText } = render(
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
        </CommandLogProvider>
      </Provider>
    );

    expect(getByText('Up')).toBeInTheDocument();
    expect(getByText('Down')).toBeInTheDocument();
    expect(getByText('Left')).toBeInTheDocument();
    expect(getByText('Right')).toBeInTheDocument();
  });

  it('calls correct function on button click', () => {
    const onUp = jest.fn();
    const onDown = jest.fn();
    const onLeft = jest.fn();
    const onRight = jest.fn();

    const { getByText } = render(
      <Provider store={store}>
        <CommandLogProvider>
          <ControlPanel
            onUp={onUp}
            onDown={onDown}
            onLeft={onLeft}
            onRight={onRight}
            labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
            controlType="wasd"
          />
        </CommandLogProvider>
      </Provider>
    );

    fireEvent.click(getByText('Up'));
    fireEvent.click(getByText('Down'));
    fireEvent.click(getByText('Left'));
    fireEvent.click(getByText('Right'));

    expect(onUp).toHaveBeenCalled();
    expect(onDown).toHaveBeenCalled();
    expect(onLeft).toHaveBeenCalled();
    expect(onRight).toHaveBeenCalled();
  });

  it('calls correct function on key press', () => {
    const onUp = jest.fn();
    const onDown = jest.fn();
    const onLeft = jest.fn();
    const onRight = jest.fn();

    render(
      <Provider store={store}>
        <CommandLogProvider>
          <ControlPanel
            onUp={onUp}
            onDown={onDown}
            onLeft={onLeft}
            onRight={onRight}
            labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
            controlType="wasd"
          />
        </CommandLogProvider>
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'w' });
    fireEvent.keyUp(window, { key: 'w' });
    fireEvent.keyDown(window, { key: 's' });
    fireEvent.keyUp(window, { key: 's' });
    fireEvent.keyDown(window, { key: 'a' });
    fireEvent.keyUp(window, { key: 'a' });
    fireEvent.keyDown(window, { key: 'd' });
    fireEvent.keyUp(window, { key: 'd' });

    expect(onUp).toHaveBeenCalled();
    expect(onDown).toHaveBeenCalled();
    expect(onLeft).toHaveBeenCalled();
    expect(onRight).toHaveBeenCalled();
  });

  it('cleans up event listeners on unmount', () => {
    const onUp = jest.fn();
    const onDown = jest.fn();
    const onLeft = jest.fn();
    const onRight = jest.fn();

    const { unmount } = render(
      <Provider store={store}>
        <CommandLogProvider>
          <ControlPanel
            onUp={onUp}
            onDown={onDown}
            onLeft={onLeft}
            onRight={onRight}
            labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
            controlType="wasd"
          />
        </CommandLogProvider>
      </Provider>
    );

    const removeEventListenerSpy = jest.spyOn(window, 'removeEventListener');
    unmount();
    expect(removeEventListenerSpy).toHaveBeenCalledTimes(2);
  });

  it('handles different control types', () => {
    const onUp = jest.fn();
    const onDown = jest.fn();
    const onLeft = jest.fn();
    const onRight = jest.fn();

    const { rerender } = render(
      <Provider store={store}>
        <CommandLogProvider>
          <ControlPanel
            onUp={onUp}
            onDown={onDown}
            onLeft={onLeft}
            onRight={onRight}
            labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
            controlType="wasd"
          />
        </CommandLogProvider>
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'w' });
    fireEvent.keyUp(window, { key: 'w' });
    expect(onUp).toHaveBeenCalled();

    rerender(
      <Provider store={store}>
        <CommandLogProvider>
          <ControlPanel
            onUp={onUp}
            onDown={onDown}
            onLeft={onLeft}
            onRight={onRight}
            labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
            controlType="arrows"
          />
        </CommandLogProvider>
      </Provider>
    );

    fireEvent.keyDown(window, { key: 'ArrowUp' });
    fireEvent.keyUp(window, { key: 'ArrowUp' });
    expect(onUp).toHaveBeenCalled();
  });
});
