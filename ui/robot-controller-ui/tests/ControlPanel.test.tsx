// File: tests/ControlPanel.test.tsx
import React from 'react';
import { render, fireEvent, screen } from '@testing-library/react';
import ControlPanel from '../src/components/ControlPanel';

describe('ControlPanel', () => {
  beforeAll(() => {
    console.log('Starting ControlPanel tests...');
  });

  beforeEach(() => {
    console.log('Setting up for a new test...');
  });

  afterEach(() => {
    console.log('Cleaning up after test...');
  });

  afterAll(() => {
    console.log('ControlPanel tests completed.');
  });

  it('renders ControlPanel with directional buttons', () => {
    console.log('Test: renders ControlPanel with directional buttons - Start');
    const mockOnUp = jest.fn();
    const mockOnDown = jest.fn();
    const mockOnLeft = jest.fn();
    const mockOnRight = jest.fn();

    render(
      <ControlPanel
        onUp={mockOnUp}
        onDown={mockOnDown}
        onLeft={mockOnLeft}
        onRight={mockOnRight}
        labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
        controlType="wasd"
      />
    );

    // Check if buttons are rendered with correct labels
    expect(screen.getByText('Up')).toBeInTheDocument();
    console.log('Test: Up button is in the document');
    expect(screen.getByText('Down')).toBeInTheDocument();
    console.log('Test: Down button is in the document');
    expect(screen.getByText('Left')).toBeInTheDocument();
    console.log('Test: Left button is in the document');
    expect(screen.getByText('Right')).toBeInTheDocument();
    console.log('Test: Right button is in the document');
    console.log('Test: renders ControlPanel with directional buttons - End');
  });

  it('calls correct function on button click', () => {
    console.log('Test: calls correct function on button click - Start');
    const mockOnUp = jest.fn();
    const mockOnDown = jest.fn();
    const mockOnLeft = jest.fn();
    const mockOnRight = jest.fn();

    render(
      <ControlPanel
        onUp={mockOnUp}
        onDown={mockOnDown}
        onLeft={mockOnLeft}
        onRight={mockOnRight}
        labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
        controlType="wasd"
      />
    );

    // Simulate button clicks
    fireEvent.click(screen.getByText('Up'));
    expect(mockOnUp).toHaveBeenCalled();
    console.log('Test: mockOnUp function called');

    fireEvent.click(screen.getByText('Down'));
    expect(mockOnDown).toHaveBeenCalled();
    console.log('Test: mockOnDown function called');

    fireEvent.click(screen.getByText('Left'));
    expect(mockOnLeft).toHaveBeenCalled();
    console.log('Test: mockOnLeft function called');

    fireEvent.click(screen.getByText('Right'));
    expect(mockOnRight).toHaveBeenCalled();
    console.log('Test: mockOnRight function called');
    console.log('Test: calls correct function on button click - End');
  });

  it('calls correct function on key press', () => {
    console.log('Test: calls correct function on key press - Start');
    const mockOnUp = jest.fn();
    const mockOnDown = jest.fn();
    const mockOnLeft = jest.fn();
    const mockOnRight = jest.fn();

    render(
      <ControlPanel
        onUp={mockOnUp}
        onDown={mockOnDown}
        onLeft={mockOnLeft}
        onRight={mockOnRight}
        labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
        controlType="wasd"
      />
    );

    // Simulate key presses
    fireEvent.keyDown(window, { key: 'w' });
    expect(mockOnUp).toHaveBeenCalled();
    console.log('Test: mockOnUp function called on key press w');

    fireEvent.keyDown(window, { key: 's' });
    expect(mockOnDown).toHaveBeenCalled();
    console.log('Test: mockOnDown function called on key press s');

    fireEvent.keyDown(window, { key: 'a' });
    expect(mockOnLeft).toHaveBeenCalled();
    console.log('Test: mockOnLeft function called on key press a');

    fireEvent.keyDown(window, { key: 'd' });
    expect(mockOnRight).toHaveBeenCalled();
    console.log('Test: mockOnRight function called on key press d');

    // Simulate unexpected key press
    fireEvent.keyDown(window, { key: 'x' });
    expect(mockOnUp).not.toHaveBeenCalledWith();
    expect(mockOnDown).not.toHaveBeenCalledWith();
    expect(mockOnLeft).not.toHaveBeenCalledWith();
    expect(mockOnRight).not.toHaveBeenCalledWith();
    console.log('Test: No functions called on unexpected key press x');
    console.log('Test: calls correct function on key press - End');
  });

  it('cleans up event listeners on unmount', () => {
    console.log('Test: cleans up event listeners on unmount - Start');
    const mockOnUp = jest.fn();
    const mockOnDown = jest.fn();
    const mockOnLeft = jest.fn();
    const mockOnRight = jest.fn();

    const { unmount } = render(
      <ControlPanel
        onUp={mockOnUp}
        onDown={mockOnDown}
        onLeft={mockOnLeft}
        onRight={mockOnRight}
        labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
        controlType="wasd"
      />
    );

    // Unmount the component and ensure no event listeners are called
    unmount();
    fireEvent.keyDown(window, { key: 'w' });
    expect(mockOnUp).not.toHaveBeenCalled();
    console.log('Test: mockOnUp function not called after unmount');
    console.log('Test: cleans up event listeners on unmount - End');
  });

  it('handles different control types', () => {
    console.log('Test: handles different control types - Start');
    const mockOnUp = jest.fn();
    const mockOnDown = jest.fn();
    const mockOnLeft = jest.fn();
    const mockOnRight = jest.fn();

    render(
      <ControlPanel
        onUp={mockOnUp}
        onDown={mockOnDown}
        onLeft={mockOnLeft}
        onRight={mockOnRight}
        labels={{ up: 'Up', down: 'Down', left: 'Left', right: 'Right' }}
        controlType="arrows"
      />
    );

    // Simulate key presses for arrow keys
    fireEvent.keyDown(window, { key: 'ArrowUp' });
    expect(mockOnUp).toHaveBeenCalled();
    console.log('Test: mockOnUp function called on key press ArrowUp');

    fireEvent.keyDown(window, { key: 'ArrowDown' });
    expect(mockOnDown).toHaveBeenCalled();
    console.log('Test: mockOnDown function called on key press ArrowDown');

    fireEvent.keyDown(window, { key: 'ArrowLeft' });
    expect(mockOnLeft).toHaveBeenCalled();
    console.log('Test: mockOnLeft function called on key press ArrowLeft');

    fireEvent.keyDown(window, { key: 'ArrowRight' });
    expect(mockOnRight).toHaveBeenCalled();
    console.log('Test: mockOnRight function called on key press ArrowRight');
    console.log('Test: handles different control types - End');
  });
});
