"""video.face_recognition
==========================

Lightweight face detection and recognition utilities used by the video
streaming server.  The goal is to offer a dependable backend feature that can
highlight faces in the outgoing MJPEG stream and attach names for known
individuals without pulling in heavyweight dependencies such as ``dlib``.

Key characteristics
-------------------

* Relies solely on OpenCV (already a dependency of the video stack).
* Loads reference images from a configurable directory and generates a very
  small embedding for each face (normalised grayscale vector).
* Uses Haar cascade detection with graceful fallbacks when the cascade or
  OpenCV are unavailable.
* Thread-safe: the recognition model can be reloaded while frames are being
  processed.

Environment variables
---------------------

``KNOWN_FACES_DIR``
    Optional path containing sub-directories (one per person) with reference
    images.  Defaults to ``video/known_faces`` relative to this module.

``FACE_RECOGNITION_THRESHOLD``
    Cosine-similarity threshold (0..1) that determines whether a detected face
    matches one of the stored embeddings.  Higher values mean stricter
    matching.  Default: ``0.6``.

``FACE_CASCADE_PATH``
    Optional override for the Haar cascade XML file.  When omitted the standard
    ``haarcascade_frontalface_default.xml`` shipped with OpenCV is used.
"""

from __future__ import annotations

import logging
import os
import threading
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:  # pragma: no cover - import guard exercised indirectly in tests
    import cv2  # type: ignore
except Exception:  # pragma: no cover - OpenCV missing on platform
    cv2 = None  # type: ignore[assignment]
    warnings.warn(
        "OpenCV is not available. Face recognition will be disabled.",
        ImportWarning,
    )


log = logging.getLogger(__name__)


@dataclass
class Detection:
    """Structured information about a detected face."""

    bbox: Tuple[int, int, int, int]
    label: str
    confidence: float


class FaceRecognizer:
    """Detect and recognise faces in video frames.

    The class maintains a catalogue of known faces loaded from disk.  For each
    detection we compute a small normalised embedding and perform cosine
    similarity matching against the stored vectors.
    """

    _SUPPORTED_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}

    def __init__(
        self,
        known_faces_dir: Optional[os.PathLike[str] | str] = None,
        recognition_threshold: Optional[float] = None,
        cascade_path: Optional[str] = None,
    ) -> None:
        self._lock = threading.RLock()
        self._known_embeddings: List[np.ndarray] = []
        self._labels: List[str] = []

        self.enabled = cv2 is not None
        self._cascade: Optional[Any] = None

        if recognition_threshold is None:
            recognition_threshold = float(os.getenv("FACE_RECOGNITION_THRESHOLD", "0.6"))
        self._threshold = float(np.clip(recognition_threshold, 0.0, 1.0))

        if known_faces_dir is None:
            default_dir = Path(__file__).resolve().parent / "known_faces"
            known_faces_dir = os.getenv("KNOWN_FACES_DIR", str(default_dir))
        self._known_dir = Path(known_faces_dir)

        if not self.enabled:
            log.warning("Face recognition disabled – OpenCV unavailable.")
            return

        if cascade_path is None:
            cascade_path = os.getenv("FACE_CASCADE_PATH")

        if cascade_path:
            self._cascade = cv2.CascadeClassifier(cascade_path)
        else:
            cascade_file = Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"
            self._cascade = cv2.CascadeClassifier(str(cascade_file))

        if self._cascade is None or self._cascade.empty():  # pragma: no cover - depends on OpenCV data files
            self.enabled = False
            log.warning("Failed to load Haar cascade. Face detection disabled.")
            return

        loaded = self._load_known_faces()
        log.info("Face recognition ready with %d known faces from %s", loaded, self._known_dir)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    @property
    def active(self) -> bool:
        """Return ``True`` when face recognition can operate."""

        return self.enabled and self._cascade is not None and not self._cascade.empty()

    @property
    def known_faces_count(self) -> int:
        with self._lock:
            return len(self._labels)

    def reload_known_faces(self) -> int:
        """Reload the known faces directory.

        Returns
        -------
        int
            Number of reference embeddings available after reload.
        """

        if not self.active:
            return 0
        loaded = self._load_known_faces()
        log.info("Reloaded %d known faces from %s", loaded, self._known_dir)
        return loaded

    def annotate(self, frame: np.ndarray) -> Tuple[np.ndarray, List[Dict[str, object]]]:
        """Detect faces inside ``frame`` and overlay bounding boxes.

        Parameters
        ----------
        frame:
            BGR image as produced by :mod:`cv2`.

        Returns
        -------
        tuple
            A tuple containing the modified frame and a list of metadata about
            each detection.  When face recognition is inactive the original
            frame and an empty list are returned unchanged.
        """

        if not self.active:
            return frame, []

        if frame is None or frame.size == 0:  # pragma: no cover - guard clause
            return frame, []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self._detect_faces(gray)

        results: List[Dict[str, object]] = []
        if not detections:
            return frame, results

        for (x, y, w, h) in detections:
            roi = gray[y : y + h, x : x + w]
            embedding = self._compute_embedding(roi)
            label, confidence = self._match_embedding(embedding)

            display_label = label or "Unknown"
            display_conf = float(confidence) if confidence is not None else 0.0
            results.append(
                {
                    "bbox": (int(x), int(y), int(w), int(h)),
                    "label": display_label,
                    "confidence": display_conf,
                }
            )

            color = (0, 255, 255) if label else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = display_label
            cv2.putText(
                frame,
                text,
                (x, max(y - 10, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        return frame, results

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _load_known_faces(self) -> int:
        embeddings: List[np.ndarray] = []
        labels: List[str] = []

        if not self._known_dir.exists():
            log.info("Known faces directory %s does not exist; skipping load.", self._known_dir)
            with self._lock:
                self._known_embeddings = []
                self._labels = []
            return 0

        for img_path in sorted(self._known_dir.rglob("*")):
            if not img_path.is_file() or img_path.suffix.lower() not in self._SUPPORTED_EXTS:
                continue

            label = self._label_from_path(img_path)
            image = cv2.imread(str(img_path))
            if image is None:
                log.warning("Failed to read image %s; skipping.", img_path)
                continue

            face_region = self._extract_face_region(image)
            if face_region is None:
                log.warning("No face detected in %s; skipping.", img_path)
                continue

            embeddings.append(self._compute_embedding(face_region))
            labels.append(label)

        with self._lock:
            self._known_embeddings = embeddings
            self._labels = labels

        return len(labels)

    def _label_from_path(self, img_path: Path) -> str:
        if img_path.parent == self._known_dir:
            return img_path.stem
        return img_path.parent.name

    def _detect_faces(self, gray_frame: np.ndarray) -> Sequence[Tuple[int, int, int, int]]:
        if self._cascade is None:
            return []
        
        # Hardware-aware detection parameters
        try:
            import os
            is_pi4b = False
            is_jetson = False
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    model = f.read().strip()
                    if "Raspberry Pi 4" in model:
                        is_pi4b = True
                    elif "NVIDIA" in model or "Jetson" in model:
                        is_jetson = True
            
            # Optimize parameters based on hardware
            if is_pi4b:
                # Faster detection for Pi 4B (less accurate but more performant)
                scale_factor = 1.2
                min_neighbors = 4
                min_size = (50, 50)
            elif is_jetson:
                # High quality detection on Jetson
                scale_factor = 1.1
                min_neighbors = 6
                min_size = (30, 30)
            else:
                # Default balanced settings
                scale_factor = 1.1
                min_neighbors = 5
                min_size = (40, 40)
        except Exception:
            # Fallback to defaults
            scale_factor = 1.1
            min_neighbors = 5
            min_size = (40, 40)
        
        faces = self._cascade.detectMultiScale(
            gray_frame,
            scaleFactor=scale_factor,
            minNeighbors=min_neighbors,
            minSize=min_size,
        )
        if isinstance(faces, tuple):
            return faces
        return faces.tolist()

    def _extract_face_region(self, image: np.ndarray) -> Optional[np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = self._detect_faces(gray)
        if not detections:
            return gray

        x, y, w, h = max(detections, key=lambda b: b[2] * b[3])
        return gray[y : y + h, x : x + w]

    def _compute_embedding(self, face: np.ndarray) -> np.ndarray:
        if face.ndim == 3:
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(face, (100, 100))
        vector = resized.astype("float32").flatten()
        norm = np.linalg.norm(vector)
        if norm == 0:
            return vector
        return vector / norm

    def _match_embedding(self, embedding: np.ndarray) -> Tuple[Optional[str], Optional[float]]:
        with self._lock:
            if not self._known_embeddings:
                return None, None
            known = np.stack(self._known_embeddings)
            labels = list(self._labels)

        scores = known @ embedding
        best_idx = int(np.argmax(scores))
        best_score = float(scores[best_idx])

        if best_score >= self._threshold:
            return labels[best_idx], best_score
        return None, best_score


__all__ = ["FaceRecognizer", "Detection"]

