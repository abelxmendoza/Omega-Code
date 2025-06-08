import unittest
from unittest.mock import patch, call

from controllers.lighting import basic_led_test


class TestBasicLedTestScript(unittest.TestCase):
    @patch("controllers.lighting.basic_led_test.time.sleep", return_value=None)
    @patch("controllers.lighting.basic_led_test.LedControl")
    def test_run_test_invokes_patterns(self, MockLedControl, _):
        instance = MockLedControl.return_value
        basic_led_test.run_test()

        self.assertEqual(instance.color_wipe.call_count, 3)
        instance.color_wipe.assert_has_calls(
            [
                call(0xFF0000, wait_ms=100),
                call(0x00FF00, wait_ms=100),
                call(0x0000FF, wait_ms=100),
            ]
        )
        instance.theater_chase.assert_called_once_with(0xFFFFFF, wait_ms=50, iterations=5)
        instance.rainbow.assert_called_once_with(wait_ms=20, iterations=1)


if __name__ == "__main__":
    unittest.main()
