"""Unit tests for the ArUco detection helper."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from video.aruco_detection import ArucoDetector

try:  # pragma: no cover - exercised implicitly by test skip
    import cv2  # type: ignore
except Exception:  # pragma: no cover - OpenCV missing on platform
    cv2 = None  # type: ignore
    _ARUCO = None  # type: ignore
else:  # pragma: no cover - behaviour verified indirectly
    _ARUCO = getattr(cv2, "aruco", None)


@pytest.mark.skipif(cv2 is None or _ARUCO is None, reason="cv2.aruco support not available")
def test_detects_marker_when_present():
    """A generated marker should be detected and annotated."""

    if hasattr(_ARUCO, "getPredefinedDictionary"):
        dictionary = _ARUCO.getPredefinedDictionary(_ARUCO.DICT_4X4_50)
    else:  # pragma: no cover - compatibility path for older OpenCV builds
        dictionary = _ARUCO.Dictionary_get(_ARUCO.DICT_4X4_50)

    if hasattr(_ARUCO, "drawMarker"):
        marker_img = _ARUCO.drawMarker(dictionary, 7, 160)
    elif hasattr(_ARUCO, "generateImageMarker"):
        marker_img = _ARUCO.generateImageMarker(dictionary, 7, 160)
    else:  # pragma: no cover - depends on contrib build
        pytest.skip("cv2.aruco cannot generate markers on this build")

    frame = np.full((240, 240, 3), 255, dtype=np.uint8)
    start = 40
    end = start + marker_img.shape[0]
    frame[start:end, start:end] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    detector = ArucoDetector(dictionary_name="DICT_4X4_50")
    assert detector.active

    annotated, detections = detector.annotate(frame.copy())
    assert annotated is not frame
    assert len(detections) >= 1

    first = detections[0]
    assert first.marker_id == 7

    cx, cy = first.centre
    assert 100 <= cx <= 140
    assert 100 <= cy <= 140


@pytest.mark.skipif(cv2 is None or _ARUCO is None, reason="cv2.aruco support not available")
def test_invalid_dictionary_disables_detector():
    detector = ArucoDetector(dictionary_name="NOT_A_REAL_DICT")
    assert not detector.active
