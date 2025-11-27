"""Utilities for detecting ArUco fiducial markers in video frames.

The :class:`ArucoDetector` class provides a small wrapper around
``cv2.aruco`` so the rest of the video pipeline can highlight markers when
OpenCV's ArUco contrib module is available.  Detection is entirely optional –
the class gracefully degrades when the contrib module is missing or when the
requested dictionary cannot be constructed.

Environment variables
---------------------

``ARUCO_DICTIONARY``
    Name of the predefined dictionary that should be used.  Defaults to
    ``DICT_4X4_50``.

``ARUCO_MARKER_LENGTH``
    Optional physical marker length in metres.  When provided together with a
    calibration file the detector will estimate pose information for each
    marker.

``ARUCO_CALIBRATION_FILE``
    Optional ``.npz`` file containing ``camera_matrix`` and ``dist_coeffs``
    arrays.  Used when pose estimation is enabled.

``ARUCO_DRAW_AXES``
    ``1``/``true`` enables drawing a pose axis for detected markers when pose
    estimation data is available.  Disabled by default.
"""

from __future__ import annotations

import logging
import os
import warnings
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:  # pragma: no cover - import guard exercised indirectly in tests
    import cv2  # type: ignore
except Exception:  # pragma: no cover - OpenCV missing on platform
    cv2 = None  # type: ignore[assignment]
    _ARUCO = None  # type: ignore[assignment]
    warnings.warn(
        "OpenCV is not available. ArUco detection will be disabled.",
        ImportWarning,
    )
else:  # pragma: no cover - behaviour tested indirectly
    _ARUCO = getattr(cv2, "aruco", None)
    if _ARUCO is None:
        warnings.warn(
            "cv2.aruco is not available. Install opencv-contrib-python to "
            "enable ArUco marker detection.",
            ImportWarning,
        )


log = logging.getLogger(__name__)


def _detect_hardware() -> dict:
    """Detect hardware for ArUco optimizations."""
    is_pi4b = False
    is_jetson = False
    
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().strip()
                if "Raspberry Pi 4" in model:
                    is_pi4b = True
                elif "NVIDIA" in model or "Jetson" in model:
                    is_jetson = True
    except Exception:
        pass
    
    return {"is_pi4b": is_pi4b, "is_jetson": is_jetson}


_hardware = _detect_hardware()


def _normalise_bool(value: Optional[str], default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "on", "yes"}


def _as_tuple(sequence: Sequence[float]) -> Tuple[float, float]:
    return float(sequence[0]), float(sequence[1])


@dataclass(frozen=True)
class ArucoDetection:
    """Structured data representing a detected marker."""

    marker_id: int
    corners: Tuple[Tuple[float, float], ...]
    centre: Tuple[float, float]
    rvec: Optional[Tuple[float, float, float]] = None
    tvec: Optional[Tuple[float, float, float]] = None


class ArucoDetector:
    """Detect and annotate ArUco markers inside BGR frames."""

    _DEFAULT_DICTIONARY = "DICT_4X4_50"

    def __init__(
        self,
        dictionary_name: Optional[str] = None,
        marker_length: Optional[float] = None,
        calibration_file: Optional[str] = None,
        draw_axes: Optional[bool] = None,
    ) -> None:
        self.enabled = cv2 is not None and _ARUCO is not None
        self._dictionary_name = (
            dictionary_name
            or os.getenv("ARUCO_DICTIONARY", self._DEFAULT_DICTIONARY)
        )
        if draw_axes is None:
            self._draw_axes = _normalise_bool(os.getenv("ARUCO_DRAW_AXES"))
        else:
            self._draw_axes = bool(draw_axes)

        if marker_length is None:
            marker_length_env = os.getenv("ARUCO_MARKER_LENGTH")
            if marker_length_env:
                marker_length_env = marker_length_env.strip()
                if marker_length_env:
                    try:
                        marker_length = float(marker_length_env)
                    except ValueError:
                        log.warning("Invalid ARUCO_MARKER_LENGTH '%s' – ignoring", marker_length_env)
                        marker_length = None

        if isinstance(marker_length, str):
            marker_length = marker_length.strip()
            marker_length = float(marker_length) if marker_length else None

        self._marker_length: Optional[float]
        if isinstance(marker_length, (int, float)):
            self._marker_length = float(marker_length)
        else:
            self._marker_length = None

        self._calibration_file = calibration_file or os.getenv("ARUCO_CALIBRATION_FILE")
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None

        self._dictionary = None
        self._detector = None
        self._parameters = None

        if not self.enabled:
            log.debug("ArUco detector disabled – OpenCV contrib module unavailable.")
            return

        try:
            self._dictionary = self._load_dictionary(self._dictionary_name)
        except ValueError as exc:
            log.warning("ArUco dictionary error: %s", exc)
            self.enabled = False
            return

        self._parameters = self._create_parameters()
        self._detector = self._create_detector()

        if self._calibration_file:
            self._load_calibration(self._calibration_file)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self.enabled and self._dictionary is not None and (
            self._detector is not None or self._parameters is not None
        )

    def annotate(self, frame: np.ndarray) -> Tuple[np.ndarray, List[ArucoDetection]]:
        """Detect markers and annotate the frame.

        Parameters
        ----------
        frame:
            Frame in BGR colour space.

        Returns
        -------
        tuple
            Modified frame and structured detection metadata.  When the
            detector is inactive the original frame and an empty list are
            returned.
        """

        if not self.active:
            return frame, []

        if frame is None or frame.size == 0:  # pragma: no cover - defensive
            return frame, []

        # Hardware-aware grayscale conversion (can skip on Jetson if already grayscale)
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        
        corners, ids = self._detect(gray)
        if not corners or ids is None:
            return frame, []

        _ARUCO.drawDetectedMarkers(frame, corners, ids)

        detections: List[ArucoDetection] = []
        pose = self._estimate_pose(corners)

        for idx, marker_corners in enumerate(corners):
            marker_id = int(ids[idx][0]) if ids is not None else -1
            pts = marker_corners.reshape(-1, 2)
            centre = np.mean(pts, axis=0)
            corners_tuple: Tuple[Tuple[float, float], ...] = tuple(_as_tuple(p) for p in pts)

            rvec = tvec = None
            if pose is not None:
                rvecs, tvecs = pose
                rvec = _as_tuple(rvecs[idx].ravel())  # type: ignore[index]
                tvec = _as_tuple(tvecs[idx].ravel())  # type: ignore[index]
                if (
                    self._draw_axes
                    and self._marker_length
                    and self._camera_matrix is not None
                    and hasattr(_ARUCO, "drawAxis")
                ):
                    dist_for_axis = (
                        self._dist_coeffs
                        if self._dist_coeffs is not None
                        else np.zeros((1, 5), dtype=float)
                    )
                    _ARUCO.drawAxis(
                        frame,
                        self._camera_matrix,
                        dist_for_axis,
                        rvecs[idx],
                        tvecs[idx],
                        self._marker_length,
                    )

            detections.append(
                ArucoDetection(
                    marker_id=marker_id,
                    corners=corners_tuple,
                    centre=_as_tuple(centre),
                    rvec=rvec,
                    tvec=tvec,
                )
            )

        return frame, detections

    @property
    def dictionary_name(self) -> str:
        return self._dictionary_name

    @property
    def pose_estimation_enabled(self) -> bool:
        return self._marker_length is not None and self._camera_matrix is not None

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _load_dictionary(self, name: str):
        if _ARUCO is None:
            raise ValueError("cv2.aruco is unavailable")

        resolved = name.strip().upper()
        # ``getPredefinedDictionary`` is preferred, but ``Dictionary_get``
        # exists on older OpenCV releases.  Try both for compatibility.
        if hasattr(_ARUCO, "getPredefinedDictionary"):
            try:
                return _ARUCO.getPredefinedDictionary(getattr(_ARUCO, resolved))
            except AttributeError as exc:
                raise ValueError(f"Unknown ArUco dictionary '{name}'") from exc

        if hasattr(_ARUCO, "Dictionary_get"):
            try:
                return _ARUCO.Dictionary_get(getattr(_ARUCO, resolved))
            except AttributeError as exc:
                raise ValueError(f"Unknown ArUco dictionary '{name}'") from exc

        raise ValueError("cv2.aruco dictionary helpers not available")

    def _create_parameters(self):
        if _ARUCO is None:
            return None
        factory = getattr(_ARUCO, "DetectorParameters_create", None)
        if callable(factory):
            return factory()
        if hasattr(_ARUCO, "DetectorParameters"):
            return _ARUCO.DetectorParameters()
        return None

    def _create_detector(self):
        if _ARUCO is None or self._dictionary is None:
            return None
        detector_cls = getattr(_ARUCO, "ArucoDetector", None)
        if detector_cls is not None and self._parameters is not None:
            try:
                return detector_cls(self._dictionary, self._parameters)
            except Exception as exc:  # pragma: no cover - depends on OpenCV build
                log.debug("Failed to create ArUco detector class: %s", exc)
        return None

    def _detect(self, gray_frame: np.ndarray):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray_frame)
            return corners, ids
        corners, ids, _ = _ARUCO.detectMarkers(
            gray_frame,
            self._dictionary,
            parameters=self._parameters,
        )
        return corners, ids

    def _load_calibration(self, path: str) -> None:
        try:
            data = np.load(path)
        except Exception as exc:
            log.warning("Failed to load ArUco calibration file %s: %s", path, exc)
            return

        matrix = data.get("camera_matrix")
        dist = data.get("dist_coeffs")
        if matrix is None or dist is None:
            log.warning("Calibration file %s missing camera_matrix/dist_coeffs", path)
            return

        self._camera_matrix = np.asarray(matrix, dtype=float)
        self._dist_coeffs = np.asarray(dist, dtype=float)

    def _estimate_pose(self, corners: Iterable[np.ndarray]):
        if (
            self._marker_length is None
            or self._camera_matrix is None
            or _ARUCO is None
        ):
            return None
        dist = (
            self._dist_coeffs
            if self._dist_coeffs is not None
            else np.zeros((1, 5), dtype=float)
        )
        try:
            rvecs, tvecs, _ = _ARUCO.estimatePoseSingleMarkers(
                list(corners),
                self._marker_length,
                self._camera_matrix,
                dist,
            )
            return rvecs, tvecs
        except Exception as exc:  # pragma: no cover - depends on OpenCV build
            log.debug("Pose estimation failed: %s", exc)
            return None


__all__ = ["ArucoDetector", "ArucoDetection"]

