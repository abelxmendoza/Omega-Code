# File: /Users/abel_elreaper/Desktop/Omega-Code/servers/robot-controller-backend/tests/led_control_test.py

import unittest
from led_control import Led

class TestLedControl(unittest.TestCase):
    def setUp(self):
        self.led = Led()

    def test_color_wipe(self):
        # This method will test the colorWipe functionality
        try:
            self.led.colorWipe(0xFF0000)  # Red color
        except Exception as e:
            self.fail(f"colorWipe method raised an exception: {e}")

    def test_theater_chase(self):
        # This method will test the theaterChase functionality
        try:
            self.led.theaterChase(0x00FF00)  # Green color
        except Exception as e:
            self.fail(f"theaterChase method raised an exception: {e}")

    def test_rainbow(self):
        # This method will test the rainbow functionality
        try:
            self.led.rainbow()
        except Exception as e:
            self.fail(f"rainbow method raised an exception: {e}")

    def test_set_led_single(self):
        # This method will test the setLed functionality with 'single' mode
        try:
            self.led.setLed(0x0000FF, 'single', 'static', 1000)  # Blue color
        except Exception as e:
            self.fail(f"setLed method raised an exception: {e}")

    def test_set_led_multi(self):
        # This method will test the setLed functionality with 'multi' mode
        try:
            self.led.setLed(0x0000FF, 'multi', 'blink', 1000)  # Blue color
        except Exception as e:
            self.fail(f"setLed method raised an exception: {e}")

    def test_set_led_two(self):
        # This method will test the setLed functionality with 'two' mode
        try:
            self.led.setLed(0x0000FF, 'two', 'fade', 1000)  # Blue color
        except Exception as e:
            self.fail(f"setLed method raised an exception: {e}")

if __name__ == '__main__':
    unittest.main()
