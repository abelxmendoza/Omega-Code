import unittest
from std_msgs.msg import Float32
import battery_monitor

class TestBatteryMonitor(unittest.TestCase):

    def test_battery_callback(self):
        # Test case for battery level above threshold
        data = Float32(data=50.0)
        battery_monitor.battery_callback(data)
        # Assuming send_alert is properly mocked or tested separately

    def test_send_alert(self):
        # Mock the smtplib.SMTP object
        import smtplib
        from unittest.mock import patch, MagicMock

        with patch('smtplib.SMTP') as mock_smtp:
            instance = mock_smtp.return_value
            instance.sendmail = MagicMock()
            battery_monitor.send_alert(15)
            instance.sendmail.assert_called_once()

if __name__ == '__main__':
    unittest.main()


