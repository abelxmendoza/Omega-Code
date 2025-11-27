from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

try:  # pragma: no cover - exercised in environments without OpenCV
    import cv2  # type: ignore
except Exception:  # pragma: no cover - ensures tests skip gracefully
    cv2 = None  # type: ignore[assignment]

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from video import face_recognition as face_recognition_module


pytestmark = pytest.mark.skipif(cv2 is None, reason="OpenCV not available")


def _write_image(path: Path, value: int) -> None:
    image = np.full((50, 50, 3), value, dtype=np.uint8)
    success = cv2.imwrite(str(path), image)
    assert success, "Failed to create test image"


def test_face_recognizer_matches_known_face(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        face_recognition_module.FaceRecognizer,
        "_extract_face_region",
        lambda self, image: cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
    )

    alice_dir = tmp_path / "Alice"
    alice_dir.mkdir()
    _write_image(alice_dir / "img1.png", 120)

    recognizer = face_recognition_module.FaceRecognizer(known_faces_dir=tmp_path, recognition_threshold=0.5)

    frame = np.full((50, 50, 3), 120, dtype=np.uint8)
    monkeypatch.setattr(recognizer, "_detect_faces", lambda gray: [(0, 0, 50, 50)])

    _, detections = recognizer.annotate(frame.copy())

    assert detections
    assert detections[0]["label"] == "Alice"
    assert detections[0]["confidence"] >= 0.5


def test_face_recognizer_unknown_when_not_matching(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        face_recognition_module.FaceRecognizer,
        "_extract_face_region",
        lambda self, image: cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
    )

    alice_dir = tmp_path / "Alice"
    alice_dir.mkdir()
    _write_image(alice_dir / "img1.png", 200)

    recognizer = face_recognition_module.FaceRecognizer(known_faces_dir=tmp_path, recognition_threshold=0.5)
    monkeypatch.setattr(recognizer, "_detect_faces", lambda gray: [(0, 0, 50, 50)])

    frame = np.zeros((50, 50, 3), dtype=np.uint8)
    _, detections = recognizer.annotate(frame.copy())

    assert detections
    assert detections[0]["label"] == "Unknown"
    assert detections[0]["confidence"] < 0.5


def test_reload_known_faces_detects_new_entries(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        face_recognition_module.FaceRecognizer,
        "_extract_face_region",
        lambda self, image: cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
    )

    alice_dir = tmp_path / "Alice"
    alice_dir.mkdir()
    _write_image(alice_dir / "img1.png", 180)

    recognizer = face_recognition_module.FaceRecognizer(known_faces_dir=tmp_path, recognition_threshold=0.4)
    assert recognizer.known_faces_count == 1

    bob_dir = tmp_path / "Bob"
    bob_dir.mkdir()
    _write_image(bob_dir / "img1.png", 60)

    count = recognizer.reload_known_faces()
    assert count == 2
    assert recognizer.known_faces_count == 2
