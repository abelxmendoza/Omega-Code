import React from 'react';
import { render } from '@testing-library/react';
import { CommandLogProvider, useCommandLog } from '../src/components/CommandLogContext';

// Dummy component to consume the context
const DummyComponent: React.FC = () => {
  const { commands } = useCommandLog();
  return <div>{commands.length > 0 ? 'Commands present' : 'No commands'}</div>;
};

describe('CommandLogContext', () => {
  it('renders CommandLogContext component', () => {
    const { container } = render(
      <CommandLogProvider>
        <DummyComponent />
      </CommandLogProvider>
    );
    expect(container).toBeInTheDocument();
  });
});
