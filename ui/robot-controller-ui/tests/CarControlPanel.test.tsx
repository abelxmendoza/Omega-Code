import { render, fireEvent } from '@testing-library/react';
import CarControlPanel from '../components/CarControlPanel';
import { COMMAND } from '../control_definitions';

describe('CarControlPanel', () => {
  let sendCommandMock: jest.Mock;

  beforeEach(() => {
    sendCommandMock = jest.fn();
  });

  test('renders control buttons', () => {
    const { getByText } = render(<CarControlPanel sendCommand={sendCommandMock} />);
    expect(getByText('W')).toBeInTheDocument();
    expect(getByText('A')).toBeInTheDocument();
    expect(getByText('S')).toBeInTheDocument();
    expect(getByText('D')).toBeInTheDocument();
  });

  test('sends correct command on button click', () => {
    const { getByText } = render(<CarControlPanel sendCommand={sendCommandMock} />);

    fireEvent.click(getByText('W'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.MOVE_UP);

    fireEvent.click(getByText('A'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.MOVE_LEFT);

    fireEvent.click(getByText('S'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.MOVE_DOWN);

    fireEvent.click(getByText('D'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.MOVE_RIGHT);
  });

  test('changes button color on click', () => {
    const { getByText } = render(<CarControlPanel sendCommand={sendCommandMock} />);

    const buttonW = getByText('W');
    fireEvent.mouseDown(buttonW);
    expect(buttonW).toHaveClass('bg-gray-600');
    fireEvent.mouseUp(buttonW);
    expect(buttonW).toHaveClass('bg-gray-800');
  });
});
