from dataclasses import dataclass
from typing import Dict, List

from .utils.utils import read_yaml


@dataclass(frozen=True)
class CalibrationResult:
    """Immutable snapshot of a hand's calibration state.

    Attributes:
        motor_limits_dict: Maps motor ID → ``[lower_raw, upper_raw]`` hard limits
            as raw integers (0–4095). Values are ``None`` before calibration.
        calibrated: ``True`` when all joints have been fully calibrated.
        wrist_calibrated: ``True`` when the wrist joint has been calibrated.
    """

    motor_limits_dict: Dict[int, List]
    calibrated: bool
    wrist_calibrated: bool

    @classmethod
    def empty(cls, motor_ids: List[int]) -> "CalibrationResult":
        """Return a blank (uncalibrated) result for the given motor IDs."""
        return cls(
            motor_limits_dict={mid: [None, None] for mid in motor_ids},
            calibrated=False,
            wrist_calibrated=False,
        )

    @classmethod
    def from_calibration_path(
        cls,
        calibration_path: str,
        motor_ids: List[int],
    ) -> "CalibrationResult":
        """Load calibration state from a ``calibration.yaml`` file.

        Returns an :meth:`empty` result for any fields absent from the file.
        """
        calibration = read_yaml(calibration_path) or {}

        motor_limits_raw = calibration.get("motor_limits", {})
        motor_limits_dict = {
            mid: motor_limits_raw.get(mid, [None, None]) for mid in motor_ids
        }

        return cls(
            motor_limits_dict=motor_limits_dict,
            calibrated=calibration.get("calibrated", False) or False,
            wrist_calibrated=calibration.get("wrist_calibrated", False) or False,
        )
