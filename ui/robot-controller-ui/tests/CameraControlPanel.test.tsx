import { render, fireEvent } from '@testing-library/react';
import CameraControlPanel from '../components/CameraControlPanel';
import { COMMAND } from '../control_definitions';

describe('CameraControlPanel', () => {
  let sendCommandMock: jest.Mock;

  beforeEach(() => {
    sendCommandMock = jest.fn();
  });

  test('renders control buttons', () => {
    const { getByText } = render(<CameraControlPanel sendCommand={sendCommandMock} />);
    expect(getByText('↑')).toBeInTheDocument();
    expect(getByText('←')).toBeInTheDocument();
    expect(getByText('↓')).toBeInTheDocument();
    expect(getByText('→')).toBeInTheDocument();
  });

  test('sends correct command on button click', () => {
    const { getByText } = render(<CameraControlPanel sendCommand={sendCommandMock} />);

    fireEvent.click(getByText('↑'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.CMD_SERVO_VERTICAL, 10);

    fireEvent.click(getByText('←'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.CMD_SERVO_HORIZONTAL, 10);

    fireEvent.click(getByText('↓'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.CMD_SERVO_VERTICAL, -10);

    fireEvent.click(getByText('→'));
    expect(sendCommandMock).toHaveBeenCalledWith(COMMAND.CMD_SERVO_HORIZONTAL, -10);
  });

  test('changes button color on click', () => {
    const { getByText } = render(<CameraControlPanel sendCommand={sendCommandMock} />);

    const buttonUp = getByText('↑');
    fireEvent.mouseDown(buttonUp);
    expect(buttonUp).toHaveClass('bg-gray-600');
    fireEvent.mouseUp(buttonUp);
    expect(buttonUp).toHaveClass('bg-gray-800');
  });
});
