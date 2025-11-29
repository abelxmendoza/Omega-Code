# File: /Omega-Code/servers/robot_controller_backend/tests/integration/video/test_video_server_integration.py

import unittest
import sys
import os
from unittest.mock import patch, MagicMock

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore

try:
    from video.video_server import app, VideoStreaming
except ImportError:
    app = None
    VideoStreaming = None

class TestVideoServerIntegration(unittest.TestCase):
    def setUp(self):
        self.app = app.test_client()
        self.app.testing = True

    @unittest.skipIf(cv2 is None, "OpenCV not installed")
    @patch('video_server.cv2.VideoCapture')
    @patch('video_server.cv2.CascadeClassifier')
    def test_video_feed(self, mock_cascade, mock_capture):
        mock_capture.return_value.read.return_value = (True, MagicMock())
        response = self.app.get('/video_feed')
        self.assertEqual(response.status_code, 200)
        self.assertIn(b'--frame\r\n', response.data)

if __name__ == '__main__':
    unittest.main()

