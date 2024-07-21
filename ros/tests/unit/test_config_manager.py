# File: /Omega-Code/ros/tests/unit/test_config_manager.py

import unittest
import os
from unittest.mock import patch, mock_open
from config_manager import load_config, apply_config

class TestConfigManager(unittest.TestCase):

    @patch('builtins.open', new_callable=mock_open, read_data='{"setting": "value"}')
    def test_load_config(self, mock_file):
        config = load_config('test_environment')
        self.assertEqual(config['setting'], 'value')

    @patch('config_manager.apply_config')
    def test_apply_config(self, mock_apply_config):
        config = {'setting': 'value'}
        apply_config(config)
        mock_apply_config.assert_called_with(config)

if __name__ == '__main__':
    unittest.main()

