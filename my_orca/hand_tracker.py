"""Camera-based hand tracking using MediaPipe Hands (Tasks API, mp 0.10+).

Captures 21 hand landmarks (x, y, z) in real-time from a webcam.
"""

import os
import time
import urllib.request
from collections import deque
from typing import Optional

import cv2
import numpy as np

try:
    import mediapipe as mp
    from mediapipe.tasks.python import vision
    from mediapipe.tasks.python.vision import RunningMode
except ImportError:
    mp = None

# MediaPipe hand landmark indices
LANDMARK = {
    "wrist": 0,
    "thumb_cmc": 1, "thumb_mcp": 2, "thumb_ip": 3, "thumb_tip": 4,
    "index_mcp": 5, "index_pip": 6, "index_dip": 7, "index_tip": 8,
    "middle_mcp": 9, "middle_pip": 10, "middle_dip": 11, "middle_tip": 12,
    "ring_mcp": 13, "ring_pip": 14, "ring_dip": 15, "ring_tip": 16,
    "pinky_mcp": 17, "pinky_pip": 18, "pinky_dip": 19, "pinky_tip": 20,
}

# Hand connections (21 landmarks → line pairs) for drawing
_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),        # index
    (0, 9), (9, 10), (10, 11), (11, 12),   # middle
    (0, 13), (13, 14), (14, 15), (15, 16),  # ring
    (0, 17), (17, 18), (18, 19), (19, 20), # pinky
    (5, 9), (9, 13), (13, 17),              # palm
]

_MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/"
              "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task")

def _download_model() -> str:
    """Download the hand landmarker model if not present. Returns local path."""
    cache_dir = os.path.join(os.path.dirname(__file__), ".cache")
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, "hand_landmarker.task")
    if not os.path.exists(path):
        print(f"[HandTracker] Downloading hand_landmarker.task ...")
        urllib.request.urlretrieve(_MODEL_URL, path)
        print(f"[HandTracker] Model saved to {path}")
    return path


def _draw_landmarks(frame: np.ndarray, landmarks_px: np.ndarray):
    """Draw landmarks and connections on the frame (no mp.solutions.drawing_utils)."""
    h, w = frame.shape[:2]
    for x, y, _ in landmarks_px:
        cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)
    for i, j in _CONNECTIONS:
        x1, y1 = int(landmarks_px[i, 0]), int(landmarks_px[i, 1])
        x2, y2 = int(landmarks_px[j, 0]), int(landmarks_px[j, 1])
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)


class HandTracker:
    """Real-time hand landmark tracker backed by MediaPipe Tasks API."""

    def __init__(
        self,
        camera_id: int = 0,
        max_num_hands: int = 1,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.6,
        smoothing_window: int = 2,
    ):
        if mp is None:
            raise ImportError(
                "mediapipe is required. Install with: pip install mediapipe"
            )

        model_path = _download_model()
        options = vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.VIDEO,
            num_hands=max_num_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_hand_presence_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)

        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self._landmark_history: deque = deque(maxlen=smoothing_window)
        self._latest_landmarks: Optional[np.ndarray] = None
        self._latest_world_landmarks: Optional[np.ndarray] = None
        self._frame: Optional[np.ndarray] = None
        self._frame_count: int = 0

    @property
    def frame(self) -> Optional[np.ndarray]:
        return self._frame

    @property
    def landmarks(self) -> Optional[np.ndarray]:
        """Return latest smoothed landmarks as (21, 3) pixel array or None."""
        return self._latest_landmarks

    @property
    def world_landmarks(self) -> Optional[np.ndarray]:
        """Return latest world landmarks as (21, 3) array (meters) or None."""
        return self._latest_world_landmarks

    def read_frame(self) -> bool:
        """Capture and process one frame. Returns True if a hand was detected."""
        ret, frame = self.cap.read()
        if not ret:
            return False

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        self._frame = frame
        self._frame_count += 1

        result = self._landmarker.detect_for_video(mp_image, self._frame_count)

        if result.hand_landmarks:
            h, w = frame.shape[:2]
            landmarks_px = np.array(
                [[lm.x * w, lm.y * h, lm.z * w] for lm in result.hand_landmarks[0]],
                dtype=np.float64,
            )

            self._landmark_history.append(landmarks_px)
            stacked = np.stack(list(self._landmark_history), axis=0)
            self._latest_landmarks = np.mean(stacked, axis=0)

            if result.hand_world_landmarks:
                wl = result.hand_world_landmarks[0]
                self._latest_world_landmarks = np.array(
                    [[lm.x, lm.y, lm.z] for lm in wl], dtype=np.float64
                )

            # Draw landmarks manually
            _draw_landmarks(frame, landmarks_px)
            return True

        return False

    def draw_info(self, text_lines: list[str], color=(0, 255, 0)):
        """Overlay text lines on the current frame."""
        if self._frame is None:
            return
        for i, line in enumerate(text_lines):
            cv2.putText(
                self._frame, line, (10, 30 + i * 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
            )

    def show(self, window_name: str = "Hand Tracker", wait_ms: int = 1) -> int:
        """Display the frame. Returns the raw keycode (or -1 if no key)."""
        if self._frame is not None:
            cv2.imshow(window_name, self._frame)
        return cv2.waitKey(wait_ms) & 0xFF

    def close(self):
        """Release camera and close windows."""
        self.cap.release()
        cv2.destroyAllWindows()
        self._landmarker.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
