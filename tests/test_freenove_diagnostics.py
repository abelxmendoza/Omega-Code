"""Hardware-free tests for the read-only Freenove diagnostic."""
import importlib.util
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("freenove_diagnostics", ROOT / "tools" / "freenove_diagnostics.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class FakeBus:
    def __init__(self, number):
        self.number = number
        self.calls = []
        self.closed = False
    def read_byte_data(self, address, register):
        self.calls.append((address, register))
        return {0: 0x11, 0xFE: 121}[register]
    def close(self):
        self.closed = True

class DiagnosticsTests(unittest.TestCase):
    def test_read_only_and_close(self):
        instances = []
        def factory(number):
            bus = FakeBus(number)
            instances.append(bus)
            return bus
        result = module.diagnose(factory)
        self.assertEqual(result["status"], "reachable")
        self.assertTrue(result["sleeping"])
        self.assertEqual(instances[0].calls, [(0x40, 0), (0x40, 0xFE)])
        self.assertTrue(instances[0].closed)
    def test_missing_device(self):
        class Missing(FakeBus):
            def read_byte_data(self, address, register):
                raise OSError("Remote I/O error")
        self.assertEqual(module.diagnose(Missing)["stage"], "read_controller")
    def test_missing_bus(self):
        def fail(number):
            raise PermissionError("denied")
        self.assertEqual(module.diagnose(fail)["stage"], "open_bus")
    def test_invalid_address(self):
        with self.assertRaises(ValueError):
            module.inspect_controller(FakeBus(1), 0x80)

if __name__ == "__main__":
    unittest.main()
