"""Map MediaPipe hand landmarks to OrcaHand joint angles (degrees 0-360).

Uses true 3D geometry (palm frame, signed splay angles, weighted
MCP/PIP/DIP blending) ported from orca-feetech-STS3215 reference.
Supports optional EMA smoothing for jitter reduction.
"""

import math
from typing import Optional

import numpy as np

# MediaPipe landmark indices
_WRIST = 0
_THUMB_CMC = 1
_THUMB_MCP = 2
_THUMB_IP = 3
_THUMB_TIP = 4
_INDEX_MCP = 5
_INDEX_PIP = 6
_INDEX_DIP = 7
_INDEX_TIP = 8
_MIDDLE_MCP = 9
_MIDDLE_PIP = 10
_MIDDLE_DIP = 11
_MIDDLE_TIP = 12
_RING_MCP = 13
_RING_PIP = 14
_RING_DIP = 15
_RING_TIP = 16
_PINKY_MCP = 17
_PINKY_PIP = 18
_PINKY_DIP = 19
_PINKY_TIP = 20


# ---- vector helpers ----------------------------------------------------------

def _unit(v: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > eps else np.zeros_like(v)


def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Angle in degrees between two vectors."""
    d = np.clip(np.dot(_unit(v1), _unit(v2)), -1.0, 1.0)
    return float(np.degrees(np.arccos(d)))


def _signed_angle_2d(v1: np.ndarray, v2: np.ndarray) -> float:
    """Signed angle in degrees from v1 to v2 in the XY plane."""
    a1 = math.atan2(v1[1], v1[0])
    a2 = math.atan2(v2[1], v2[0])
    return float(np.degrees(a2 - a1))


def _flexion_from_angle(angle_abc_deg: float) -> float:
    """Convert internal joint angle to flexion measure.
    Straight finger ~180 deg -> flexion = 180 - angle, clamped >= 0.
    """
    return max(0.0, 180.0 - float(angle_abc_deg))


def _signed_angle_about_axis(v_from: np.ndarray, v_to: np.ndarray, axis: np.ndarray) -> float:
    """Signed angle from v_from to v_to about axis (degrees)."""
    a = _unit(axis)
    v1 = _unit(v_from)
    v2 = _unit(v_to)
    cross = np.cross(v1, v2)
    sin_term = float(np.dot(a, cross))
    cos_term = float(np.clip(np.dot(v1, v2), -1.0, 1.0))
    return float(np.degrees(np.arctan2(sin_term, cos_term)))


# ---- main mapper class -------------------------------------------------------

class HandToOrcaMapper:
    """Compute ORCA Hand joint angles (0-360) from MediaPipe 3D landmarks.

    Uses a palm-frame approach for robust 3D geometry.
    Supports EMA smoothing for jitter reduction.

    Joint naming:
        thumb_mcp, thumb_abd, thumb_pip, thumb_dip,
        index_abd, index_mcp, index_pip,
        middle_abd, middle_mcp, middle_pip,
        ring_abd, ring_mcp, ring_pip,
        pinky_abd, pinky_mcp, pinky_pip,
        wrist
    """

    # Weighting for proximal/distal joint blending
    PROX_W_MCP, PROX_W_PIP, PROX_W_DIP = 0.45, 0.45, 0.10
    DIST_W_MCP, DIST_W_PIP, DIST_W_DIP = 0.10, 0.45, 0.45

    def __init__(self, flip_hand: bool = False, limits_dict: dict | None = None,
                 inverted_joints: set | None = None,
                 ema: float = 0.0):
        """
        Args:
            flip_hand: If True, mirror landmarks horizontally.
            limits_dict: joint_name -> [min_raw, max_raw] from calibration.
            inverted_joints: Set of joint names that are inverted.
            ema: EMA smoothing factor (0 = no smoothing, 0.75 = moderate).
        """
        self.flip_hand = flip_hand
        self.limits_dict = limits_dict
        self.inverted_joints = inverted_joints or set()
        self.ema = float(ema) if ema else 0.0
        self._prev: dict[str, float] = {}
        self._span_initialized = False
        self._ref_hand_span: float = 150.0

    # ---- smoothing -----------------------------------------------------------

    def _ema_update(self, cur: dict[str, float]) -> dict[str, float]:
        if self.ema <= 0.0 or not self._prev:
            self._prev = dict(cur)
            return dict(cur)
        out = dict(self._prev)
        alpha = float(self.ema)
        for k, v in cur.items():
            if k in out:
                out[k] = alpha * out[k] + (1.0 - alpha) * v
            else:
                out[k] = v
        self._prev = out
        return dict(out)

    # ---- output mapping ------------------------------------------------------

    def _to_output(self, joint: str, anatomical_deg: float,
                    anatomical_max: float = 100.0) -> float:
        """Map anatomical angle to servo degree (0-360), accounting for inversion."""
        if self.limits_dict and joint in self.limits_dict:
            lo_raw, hi_raw = self.limits_dict[joint]
            if joint in self.inverted_joints:
                ext_deg = (4095.0 - hi_raw) / 4095.0 * 360.0
                flex_deg = (4095.0 - lo_raw) / 4095.0 * 360.0
                lo_deg, hi_deg = ext_deg, flex_deg
            else:
                lo_deg = (lo_raw / 4095.0) * 360.0
                hi_deg = (hi_raw / 4095.0) * 360.0
        else:
            lo_deg, hi_deg = 0.0, 360.0

        frac = max(0.0, min(1.0, anatomical_deg / anatomical_max))
        return float(lo_deg + frac * (hi_deg - lo_deg))

    # ---- public API -----------------------------------------------------------

    def compute_joint_angles(
        self,
        landmarks_px: np.ndarray,
        landmarks_world: Optional[np.ndarray] = None,
    ) -> dict[str, float]:
        """Return joint_name -> servo_angle_degrees (0-360).

        Args:
            landmarks_px: (21, 3) pixel-space landmarks [x_px, y_px, z_px].
            landmarks_world: (21, 3) world landmarks in meters (optional).
        """
        if landmarks_px.shape != (21, 3):
            raise ValueError(f"Expected landmarks_px shape (21, 3), got {landmarks_px.shape}")

        lm = landmarks_px.copy()

        if self.flip_hand:
            lm[:, 0] = -lm[:, 0]

        if not self._span_initialized:
            span = np.linalg.norm(lm[_INDEX_MCP] - lm[_PINKY_MCP])
            if span > 30:
                self._ref_hand_span = span
                self._span_initialized = True

        # Build 3D palm frame
        wrist = lm[_WRIST]
        palm_x = _unit(lm[_INDEX_MCP] - lm[_PINKY_MCP])      # pinky -> index axis
        palm_y = _unit(lm[_MIDDLE_MCP] - wrist)              # wrist -> middle_mcp axis
        palm_z = _unit(np.cross(palm_x, palm_y))
        palm_y = _unit(np.cross(palm_z, palm_x))             # re-orthogonalize

        angles: dict[str, float] = {}

        # Four fingers: weighted MCP/PIP/DIP -> prox/dist
        fingers = [
            ("index", _INDEX_MCP, _INDEX_PIP, _INDEX_DIP),
            ("middle", _MIDDLE_MCP, _MIDDLE_PIP, _MIDDLE_DIP),
            ("ring", _RING_MCP, _RING_PIP, _RING_DIP),
            ("pinky", _PINKY_MCP, _PINKY_PIP, _PINKY_DIP),
        ]
        for name, mcp_i, pip_i, dip_i in fingers:
            f_mcp = _flexion_from_angle(
                _angle_between(wrist - lm[mcp_i], lm[pip_i] - lm[mcp_i]))
            f_pip = _flexion_from_angle(
                _angle_between(lm[mcp_i] - lm[pip_i], lm[dip_i] - lm[pip_i]))
            f_dip = 0.0  # MediaPipe doesn't have DIP landmarks; use PIP as proxy
            # Blend: DIP uses mostly PIP flexion (same for 2-DOF fingers)
            f_dip = f_pip * 0.9

            f_prox = (self.PROX_W_MCP * f_mcp + self.PROX_W_PIP * f_pip
                       + self.PROX_W_DIP * f_dip)
            f_dist = (self.DIST_W_MCP * f_mcp + self.DIST_W_PIP * f_pip
                       + self.DIST_W_DIP * f_dip)

            angles[f"{name}_mcp"] = self._to_output(f"{name}_mcp", f_prox)
            angles[f"{name}_pip"] = self._to_output(f"{name}_pip", f_dist)

        # Thumb: 3D geometry with root axis tilt
        thumb_root_flex = 0.0
        try:
            tilt = math.radians(30.0)
            root_axis = _unit(math.cos(tilt) * palm_z + math.sin(tilt) * (-palm_x))
            thumb_meta = lm[_THUMB_MCP] - lm[_THUMB_CMC]
            ref = palm_y - root_axis * float(np.dot(palm_y, root_axis))
            meta_p = thumb_meta - root_axis * float(np.dot(thumb_meta, root_axis))
            thumb_root_flex = abs(_signed_angle_about_axis(
                ref, meta_p, root_axis))
        except Exception:
            thumb_root_flex = 0.0

        thumb_mcp_flex = _flexion_from_angle(
            _angle_between(lm[_THUMB_CMC] - lm[_THUMB_MCP],
                            lm[_THUMB_IP] - lm[_THUMB_MCP]))
        thumb_ip_flex = _flexion_from_angle(
            _angle_between(lm[_THUMB_MCP] - lm[_THUMB_IP],
                            lm[_THUMB_TIP] - lm[_THUMB_IP]))

        angles["thumb_mcp"] = self._to_output("thumb_mcp", thumb_root_flex, anatomical_max=100.0)
        angles["thumb_abd"] = self._to_output("thumb_abd",
            self._thumb_splay(lm, palm_y, palm_z), anatomical_max=60.0)
        angles["thumb_pip"] = self._to_output("thumb_pip",
            thumb_mcp_flex + 12, anatomical_max=120.0)
        angles["thumb_dip"] = self._to_output("thumb_dip",
            thumb_ip_flex + 20, anatomical_max=132.0)

        # Finger splay (3D): project MCP->PIP into palm plane, signed about palm normal
        self._finger_splay_3d(angles, lm, palm_y, palm_z)

        # Wrist: palm orientation relative to camera
        angles["wrist"] = self._to_output("wrist",
            self._wrist_angle(lm, palm_x, palm_z), anatomical_max=100.0)

        # EMA smoothing
        if self.ema > 0.0:
            angles = self._ema_update(angles)

        return angles

    # ---- thumb splay ---------------------------------------------------------

    def _thumb_splay(self, lm: np.ndarray,
                      palm_y: np.ndarray, palm_z: np.ndarray) -> float:
        """Signed thumb splay angle about palm normal."""
        v_thumb_prox = lm[_THUMB_IP] - lm[_THUMB_MCP]
        v_thumb_prox_palm = v_thumb_prox - palm_z * float(np.dot(v_thumb_prox, palm_z))
        ref_palm = palm_y - palm_z * float(np.dot(palm_y, palm_z))
        val = _signed_angle_about_axis(ref_palm, v_thumb_prox_palm, palm_z)
        return float(np.clip(val, -45, 60))

    # ---- finger splay (3D palm-plane projection) -----------------------------

    def _finger_splay_3d(self, angles: dict[str, float],
                          lm: np.ndarray,
                          palm_y: np.ndarray, palm_z: np.ndarray) -> None:
        """Compute signed finger splay angles in the palm plane."""
        def proj_palm(v):
            return v - palm_z * float(np.dot(v, palm_z))

        v_idx = proj_palm(lm[_INDEX_PIP] - lm[_INDEX_MCP])
        v_mid = proj_palm(lm[_MIDDLE_PIP] - lm[_MIDDLE_MCP])
        v_rng = proj_palm(lm[_RING_PIP] - lm[_RING_MCP])
        v_pky = proj_palm(lm[_PINKY_PIP] - lm[_PINKY_MCP])

        # Angles relative to middle finger
        splay_index = _signed_angle_about_axis(v_mid, v_idx, palm_z)
        splay_ring = _signed_angle_about_axis(v_mid, v_rng, palm_z)
        splay_pinky = _signed_angle_about_axis(v_mid, v_pky, palm_z)
        splay_middle = _signed_angle_about_axis(palm_y, v_mid, palm_z)

        # Map to anatomical range
        ref_idx, ref_mid, ref_ring, ref_pinky = 0.0, 0.0, -5.0, -10.0

        def _map_splay(raw: float, ref: float) -> float:
            delta = raw - ref
            return float(np.clip(delta, -60, 60))

        angles["index_abd"] = self._to_output("index_abd",
            _map_splay(splay_index, ref_idx) + 30, anatomical_max=60.0)
        angles["middle_abd"] = self._to_output("middle_abd",
            _map_splay(splay_middle, ref_mid) + 30, anatomical_max=60.0)
        angles["ring_abd"] = self._to_output("ring_abd",
            _map_splay(splay_ring, ref_ring) + 30, anatomical_max=60.0)
        angles["pinky_abd"] = self._to_output("pinky_abd",
            _map_splay(splay_pinky, ref_pinky) + 30, anatomical_max=60.0)

    # ---- wrist ----------------------------------------------------------------

    def _wrist_angle(self, lm: np.ndarray,
                      palm_x: np.ndarray, palm_z: np.ndarray) -> float:
        """Estimate wrist flexion from palm normal rotation about palm_x."""
        camera_z = np.array([0.0, 0.0, 1.0], dtype=float)
        axis = palm_x
        ref = camera_z - axis * float(np.dot(camera_z, axis))
        val = palm_z - axis * float(np.dot(palm_z, axis))
        if float(np.linalg.norm(ref)) < 1e-8 or float(np.linalg.norm(val)) < 1e-8:
            return 0.0
        wrist_rad = float(_signed_angle_about_axis(ref, val, axis))
        return float(np.clip(wrist_rad, -65, 35))

    # ---- convenience ----------------------------------------------------------

    def to_orca_positions(
        self,
        landmarks_px: np.ndarray,
        landmarks_world: Optional[np.ndarray] = None,
    ) -> dict[str, float]:
        """Shortcut: compute joint angles dict for OrcaHand.set_joint_positions()."""
        return self.compute_joint_angles(landmarks_px, landmarks_world)


# ---- grasp detection ----------------------------------------------------------

def detect_grasp_state(
    joint_angles: dict[str, float],
    flex_threshold: float = 150.0,
) -> tuple[bool, float]:
    """Heuristic grasp detection based on finger flexion.

    Args:
        joint_angles: Joint angles in servo degrees (0-360).
        flex_threshold: Average flexion above which grasp is triggered.

    Returns:
        (is_grasping, grasp_strength) where grasp_strength in [0, 1].
    """
    flex_keys = [
        "index_mcp", "index_pip",
        "middle_mcp", "middle_pip",
        "ring_mcp", "ring_pip",
        "pinky_mcp", "pinky_pip",
        "thumb_pip", "thumb_dip",
    ]
    flexions = [joint_angles.get(k, 0.0) for k in flex_keys]
    avg_flex = sum(flexions) / len(flexions)

    strength = float(np.clip((avg_flex - 50.0) / 260.0, 0.0, 1.0))
    is_grasping = avg_flex > flex_threshold

    return is_grasping, strength
