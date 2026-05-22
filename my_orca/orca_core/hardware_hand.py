# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
import math
import threading
import time
from collections import deque
from threading import RLock
from typing import Dict, List, TYPE_CHECKING, Union

import numpy as np

from .base_hand import BaseHand
from .calibration import CalibrationResult
from .hand_config import OrcaHandConfig
from .hardware.motor_client import MotorClient
from .overcurrent_protection import OvercurrentProtection
from .utils.utils import auto_detect_port, get_and_choose_port, update_yaml

if TYPE_CHECKING:
    from .hardware.dynamixel_client import DynamixelClient
    from .hardware.feetech_client import FeetechClient

from .constants import (
    SUPPORTED_MOTOR_TYPES,
    MODE_MAP,
    WRIST_MODE_VALUE,
    CURRENT_BASED_POSITION,
    CURRENT,
    WRIST,
    FLEX,
    EXTEND,
    JOINTS,
    STEP,
    TINY_SLEEP,
    MOTOR_LIMITS_DICT,
    WRIST_CALIBRATED,
    CALIBRATED,
    NUM_STEPS,
    POSITION,
    STEP_SIZE,
)

from .joint_position import OrcaJointPositions


class OrcaHand(BaseHand):
    """ORCA hand class.

    Extends :class:`~orca_core.BaseHand` with a full lifecycle for a physical
    hand: connection management, torque control, multi-mode motor control,
    (automatic) calibration, and background task execution.

    The recommended usage pattern is:

    >>> from orca_core import OrcaHand, OrcaJointPositions
    >>> hand = OrcaHand()
    >>> hand.connect()
    >>> hand.init_joints()  # enables torque, calibrates if needed
    >>> hand.set_joint_positions(OrcaJointPositions({"index_mcp": 0.5}))
    >>> hand.disconnect()

    Args:
        config_path: Path to a ``config.yaml`` file. Defaults to the bundled
            model when ``None``.
    """

    config_cls = OrcaHandConfig

    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        config: OrcaHandConfig | None = None,
    ):
        super().__init__(
            config_path=config_path,
            config=config,
            calibration_path=calibration_path,
            model_version=model_version,
            model_name=model_name,
        )

        self._wrap_offsets_dict: Dict[int, float] = None
        self._motor_client: MotorClient = None
        self._motor_lock: RLock = RLock()

        self._task_thread: threading.Thread = None
        self._task_stop_event = threading.Event()
        self._lock = threading.Lock()
        self._current_task = None

        # Overcurrent protection
        self.overcurrent = OvercurrentProtection(self)

        self.calibration = CalibrationResult.from_calibration_path(
            self.config.calibration_path, self.config.motor_ids
        )
        self._sanity_check()
        self.is_calibrated(verbose=True)

    def __del__(self):
        self.disconnect()

    # ------------------------------------------------------------------
    # Calibration state — thin views onto self.calibration
    # ------------------------------------------------------------------

    @property
    def motor_limits_dict(self) -> Dict[int, list]:
        return self.calibration.motor_limits_dict

    @property
    def calibrated(self) -> bool:
        return self.calibration.calibrated

    @property
    def wrist_calibrated(self) -> bool:
        return self.calibration.wrist_calibrated

    def _create_motor_client(self) -> MotorClient:
        if self.config.motor_type == "dynamixel":
            from .hardware.dynamixel_client import DynamixelClient

            return DynamixelClient(
                self.config.motor_ids, self.config.port, self.config.baudrate
            )

        if self.config.motor_type == "feetech":
            from .hardware.feetech_client import FeetechClient

            return FeetechClient(
                self.config.motor_ids, self.config.port, self.config.baudrate
            )

        raise ValueError(
            f"Unknown motor_type: {self.config.motor_type}. Expected one of [{', '.join(SUPPORTED_MOTOR_TYPES)}]."
        )

    def connect(self) -> tuple[bool, str]:
        """Open connection to the motor bus.

        Attempts to connect on the port in ``config.yaml``. On failure it
        tries auto-detection via USB vendor ID, then falls back to an
        interactive terminal port picker. A successful connection updates
        ``config.yaml`` if the port changed.

        Returns:
            A ``(success, message)`` tuple where *success* is ``True`` on a
            successful connection.
        """
        # TODO(fracapuano): Refactor: this is basically always connecting to one port and looking at multiple ports
        try:
            self._motor_client = self._create_motor_client()
            with self._motor_lock:
                self._motor_client.connect()

            # Configure and start overcurrent monitor
            self._configure_overcurrent_from_config()
            self.overcurrent.start_monitor()

            return True, "Connection successful"
        except Exception as e:
            # 1. The port is not correct
            self._motor_client = None
            print(f"Connection failed on {self.config.port}: {str(e)}")

            chosen_port = auto_detect_port(self.config.motor_type)
            if chosen_port and chosen_port != self.config.port:
                # Don't retry the same port that just failed
                print(f"Auto-detected different port: {chosen_port} (config was {self.config.port})")
                # TODO(fracapuano): Replace replace replace this try except Exception is madness
                try:
                    self.config = dataclasses.replace(self.config, port=chosen_port)
                    self._motor_client = self._create_motor_client()
                    with self._motor_lock:
                        self._motor_client.connect()
                    update_yaml(self.config.config_path, "port", chosen_port)
                    self._configure_overcurrent_from_config()
                    self.overcurrent.start_monitor()
                    return (
                        True,
                        f"Connection successful with auto-detected port {chosen_port}",
                    )

                except Exception:
                    self._motor_client = None

            print("Please select a port from available devices:")
            chosen_port = get_and_choose_port()
            if chosen_port is None:
                return False, "Connection failed: No port selected"

            try:
                self.config = dataclasses.replace(self.config, port=chosen_port)
                self._motor_client = self._create_motor_client()
                with self._motor_lock:
                    self._motor_client.connect()
                update_yaml(self.config.config_path, "port", chosen_port)
                self._configure_overcurrent_from_config()
                self.overcurrent.start_monitor()
                return True, f"Connection successful with port {chosen_port}"
            except Exception as e2:
                self._motor_client = None
                return False, f"Connection failed with selected port: {str(e2)}"

    def disconnect(self) -> tuple[bool, str]:
        """Disable torque and close the serial connection.

        Safe to call even when the hand is already disconnected.

        Returns:
            A ``(success, message)`` tuple.
        """
        try:
            if self._motor_client is None:
                return True, "Disconnected successfully"
            with self._motor_lock:
                self.overcurrent.stop_monitor()
                self.overcurrent.reset_trips(None)
                self.disable_torque()
                time.sleep(0.1)
                self._motor_client.disconnect()
            return True, "Disconnected successfully"
        except Exception as e:
            return False, f"Disconnection failed: {str(e)}"

    def is_connected(self) -> bool:
        """Return ``True`` if the motor client is connected.

        Returns:
            Connection status as a boolean.
        """
        return self._motor_client is not None and self._motor_client.is_connected

    def enable_torque(self, motor_ids: List[int] = None):
        """Enable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to enable. Defaults to all motors.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, True)

    def disable_torque(self, motor_ids: List[int] = None):
        """Disable torque on the specified motors.

        Args:
            motor_ids: List of motor IDs to disable. Defaults to all motors.
        """
        motor_ids = self.config.motor_ids if motor_ids is None else motor_ids

        with self._motor_lock:
            self._motor_client.set_torque_enabled(motor_ids, False)

    def set_max_current(self, current: Union[float, List[float]]):
        """Set the maximum allowable current for the motors.

        Args:
            current: Either a single float applied to all motors, or a list of
                per-motor current values (mA). If a list, its length must match
                the number of configured motors.

        Raises:
            ValueError: If *current* is a list with the wrong length.
        """
        if isinstance(current, list):
            if len(current) != len(self.config.motor_ids):
                raise ValueError(
                    "Number of currents do not match the number of motors."
                )

            with self._motor_lock:
                self._motor_client.write_desired_current(self.config.motor_ids, current)

        with self._motor_lock:
            self._motor_client.write_desired_current(
                self.config.motor_ids, current * np.ones(len(self.config.motor_ids))
            )

    def set_control_mode(self, mode: str, motor_ids: List[int] = None):
        """Switch the operating mode of the specified motors.

        The wrist motor is always kept in ``multi_turn_position`` mode (4) when
        *mode* would otherwise be ``current_based_position`` (5) or
        ``current`` (0), because those modes are incompatible with the wrist
        joint's range of motion.

        Args:
            mode: One of ``"current"``, ``"velocity"``, ``"position"``,
                ``"multi_turn_position"``, or ``"current_based_position"``.
            motor_ids: Motors to reconfigure. Defaults to all motors.

        Raises:
            ValueError: If *mode* is not recognised or *motor_ids* contains
                unknown IDs.
        """
        mode_value = MODE_MAP.get(mode)
        if mode_value is None:
            raise ValueError("Invalid control mode.")

        with self._motor_lock:
            if motor_ids is None:
                motor_ids = self.config.motor_ids
            elif not all(motor_id in self.config.motor_ids for motor_id in motor_ids):
                raise ValueError("Invalid motor IDs.")

        if mode_value in (MODE_MAP[CURRENT_BASED_POSITION], MODE_MAP[CURRENT]):
            wrist_motor_id = self.config.joint_to_motor_map.get("wrist")
            if wrist_motor_id is not None:
                motor_ids_without_wrist = [
                    motor_id for motor_id in motor_ids if motor_id != wrist_motor_id
                ]
                self._motor_client.set_operating_mode(
                    motor_ids_without_wrist, mode_value
                )

                if wrist_motor_id in motor_ids:
                    self._motor_client.set_operating_mode(
                        [wrist_motor_id], WRIST_MODE_VALUE
                    )

                return

        self._motor_client.set_operating_mode(motor_ids, mode_value)

    def get_motor_pos(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read raw motor positions from the bus.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.
                Defaults to ``False`` (returns an array ordered by
                :attr:`motor_ids`).

        Returns:
            Motor positions in radians as an array or dict.
        """
        with self._motor_lock:
            motor_pos = self._motor_client.read_pos_vel_cur()[0]

            if as_dict:
                return {
                    motor_id: pos
                    for motor_id, pos in zip(self.config.motor_ids, motor_pos)
                }

            return motor_pos

    def get_motor_current(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read the present current drawn by each motor.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.

        Returns:
            Motor currents (mA) as an array, or dict.
        """
        with self._motor_lock:
            motor_current = self._motor_client.read_pos_vel_cur()[2]

            if as_dict:
                return {
                    motor_id: current
                    for motor_id, current in zip(self.config.motor_ids, motor_current)
                }

            return motor_current

    def get_motor_temp(self, as_dict: bool = False) -> Union[np.ndarray, dict]:
        """Read the present temperature of each motor.

        Args:
            as_dict: When ``True`` returns a ``dict`` keyed by motor ID.

        Returns:
            Motor temperatures in °C as an array or dict.
        """
        with self._motor_lock:
            motor_temp = self._motor_client.read_temperature()

            if as_dict:
                return {
                    motor_id: temp
                    for motor_id, temp in zip(self.config.motor_ids, motor_temp)
                }

            return motor_temp

    def _get_joint_positions(self) -> OrcaJointPositions:
        motor_pos = self.get_motor_pos()
        return OrcaJointPositions.from_dict(self._motor_to_joint_pos(motor_pos))

    def _set_joint_positions(self, joint_pos: OrcaJointPositions) -> bool:
        motor_pos = self._joint_to_motor_pos(joint_pos.as_dict())
        self._set_motor_pos(motor_pos)
        return True

    def init_joints(self, move_to_neutral: bool = True):
        """Prepare the hand for operation.

        Enables torque, sets the configured control mode and current limit,
        computes wrap offsets, and optionally moves to the neutral position.

        Calibration must be done separately via ``manual_calibrate_v2.py``.
        """
        self.enable_torque()
        self.set_control_mode(self.config.control_mode)
        self.set_max_current(self.config.max_current)

        if not self.calibrated:
            print("\033[93mWarning: Hand is not calibrated. Run: python manual_calibrate_v2.py\033[0m")

        self._compute_wrap_offsets_dict()

        if move_to_neutral:
            control_mode = self.config.control_mode
            self.set_control_mode(POSITION)
            # Use safe fast path (avoids interpolation, suspends OC, safety-checked)
            self.set_neutral_position()
            self.set_control_mode(control_mode)

    def is_calibrated(self, verbose: bool = False) -> bool:
        """Check whether all joints have been fully calibrated.

        Args:
            verbose: When ``True``, prints a warning for each uncalibrated
                motor instead of returning early.

        Returns:
            ``True`` if every motor has valid limits.
        """
        overall_calibrated = True
        uncalibrated_messages = []

        for motor_id, limits in self.motor_limits_dict.items():
            if any(limit is None for limit in limits):
                overall_calibrated = False
                if not verbose:
                    return False
                joint_name = self.config.motor_to_joint_dict.get(motor_id, "Unknown")
                uncalibrated_messages.append(
                    f"\033[93mWarning: Motor ID {motor_id} (Joint: {joint_name}) has not been fully calibrated (missing motor limits).\033[0m"
                )

        if verbose:
            for msg in uncalibrated_messages:
                print(msg)

        return overall_calibrated

    def calibrate(self, blocking: bool = True, force_wrist: bool = False):
        """Run the joint calibration routine.

        Drives each joint to its mechanical limits in the sequence defined by
        ``calibration_sequence`` in ``config.yaml``, records the motor positions at
        each limit, and persists the resulting motor limits and joint-to-motor
        ratios to ``calibration.yaml``.

        On completion ``self.calibration`` is replaced with a fresh
        :class:`~orca_core.CalibrationResult`. Partial progress is written to
        disk after every step so an interrupted run is never fully lost.
        """
        if blocking:
            self._task_stop_event.clear()
            result = self._calibrate(force_wrist=force_wrist)
            if result is not None:
                self.calibration = result
        else:
            self._start_task(self._calibrate_and_apply, force_wrist=force_wrist)

    def _build_calibration_result(
        self,
        motor_limits: Dict[int, list],
        wrist_calibrated: bool,
    ) -> CalibrationResult:
        calibrated = all(
            limits[0] is not None and limits[1] is not None
            for limits in motor_limits.values()
        )

        return CalibrationResult(
            motor_limits_dict={
                motor_id: list(limits) for motor_id, limits in motor_limits.items()
            },
            calibrated=calibrated,
            wrist_calibrated=wrist_calibrated,
        )

    def _calibrate(self, force_wrist: bool = False) -> CalibrationResult | None:
        """Execute the calibration routine and return a :class:`~orca_core.CalibrationResult`.

        Drives each joint through its mechanical limits following ``calibration_sequence``
        from ``config.yaml``, records motor positions at each limit, and persists
        the resulting motor limits and joint-to-motor ratios to ``calibration.yaml``
        after every step. Returns ``None`` on early exit (stop event triggered).

        Wrist calibration logic:
        - Wrist is calibrated independently of fingers (tracked by `wrist_calibrated` in calibration file).
        - Uses a higher calibration current.
        - If already calibrated (and calibration run is not forcing), skip wrist steps.
        - If missing from sequence, is calibrated.
        - If force_wrist=True, always include wrist in calibration steps.
        """
        wrist_in_sequence = any(
            "wrist" in step[JOINTS] for step in self.config.calibration_sequence
        )
        calibration_sequence = list(self.config.calibration_sequence)

        if self.wrist_calibrated and not force_wrist:
            if wrist_in_sequence:
                print(
                    "WARNING: Wrist is already calibrated. Skipping wrist calibration. Use --force-wrist to override."
                )
            calibration_sequence = [
                step for step in calibration_sequence if WRIST not in step[JOINTS]
            ]
        elif not wrist_in_sequence:
            # Adds wrist to calibration sequence
            calibration_sequence.append(
                {STEP: len(calibration_sequence) + 1, JOINTS: {WRIST: FLEX}}
            )
            calibration_sequence.append(
                {STEP: len(calibration_sequence) + 1, JOINTS: {WRIST: EXTEND}}
            )

        # Deep-copy current limits so per-step YAML writes reflect only the
        # joints being calibrated, not stale data from a prior incomplete run.
        motor_limits = {
            motor_id: list(limits)
            for motor_id, limits in self.calibration.motor_limits_dict.items()
        }

        self._compute_wrap_offsets_dict()

        for step in calibration_sequence:
            for joint in step[JOINTS].keys():
                motor_id = self.config.joint_to_motor_map[joint]
                motor_limits[motor_id] = [None, None]
                self._wrap_offsets_dict[motor_id] = 0.0

        motors_with_initial_offset = set()
        motors_with_final_offset = set()
        
        calibrated_joints: dict = {}

        # Calibration is always done in current-based position mode.
        self.set_control_mode(CURRENT_BASED_POSITION)
        self.set_max_current(self.config.calibration_current)

        for step in calibration_sequence:
            self.disable_torque()

            if self._task_stop_event.is_set():
                return None

            desired_increment, motor_reached_limit, directions = {}, {}, {}
            position_buffers, calibrated_joints, position_logs, current_log = (
                {},
                {},
                {},
                {},
            )

            for joint, direction in step[JOINTS].items():
                self.enable_torque(motor_ids=[self.config.joint_to_motor_map[joint]])
                print(
                    "Enabling torque for the following motor: ",
                    self.config.joint_to_motor_map[joint],
                )

                if self._task_stop_event.is_set():
                    return None

                self.set_max_current(
                    self.config.calibration_current
                    if joint != WRIST
                    else self.config.wrist_calibration_current
                )

                motor_id = self.config.joint_to_motor_map[joint]
                sign = 1 if direction == FLEX else -1
                if self.config.joint_inversion_dict.get(joint, False):
                    sign = -sign

                directions[motor_id] = sign
                position_buffers[motor_id] = deque(
                    maxlen=self.config.calibration_num_stable
                )
                position_logs[motor_id] = []
                current_log[motor_id] = []
                motor_reached_limit[motor_id] = False

                if (
                    self._motor_client.requires_offset_calibration
                    and motor_id not in motors_with_initial_offset
                ):
                    self._motor_client.calibrate_offset(motor_id, upper=(sign < 0))
                    motors_with_initial_offset.add(motor_id)

            while (
                not all(motor_reached_limit.values())
                and not self._task_stop_event.is_set()
            ):
                desired_increment = {}
                for motor_id, reached_limit in motor_reached_limit.items():
                    if not reached_limit:
                        desired_increment[motor_id] = (
                            directions[motor_id] * self.config.calibration_step_size
                        )

                self._set_motor_pos(desired_increment, rel_to_current=True)
                time.sleep(self.config.calibration_step_period)

                for motor_id in desired_increment.keys():
                    if motor_reached_limit[motor_id]:
                        continue

                    # Read single motor to avoid flooding the bus
                    _pos = None
                    _cur = None
                    if hasattr(self._motor_client, 'read_position_single'):
                        _pos = self._motor_client.read_position_single(motor_id)
                    if hasattr(self._motor_client, 'read_current_single'):
                        _cur = self._motor_client.read_current_single(motor_id)

                    if _pos is None:
                        # Fallback: read all motors
                        idx = self.config.motor_id_to_idx_dict[motor_id]
                        _pos = float(self.get_motor_pos()[idx])
                        _cur = float(self.get_motor_current()[idx])

                    position_buffers[motor_id].append(_pos)
                    position_logs[motor_id].append(_pos)
                    current_log[motor_id].append(_cur if _cur is not None else 0.0)

                    if len(
                        position_buffers[motor_id]
                    ) == self.config.calibration_num_stable and np.allclose(
                        position_buffers[motor_id],
                        position_buffers[motor_id][0],
                        atol=self.config.calibration_threshold,
                    ):
                        motor_reached_limit[motor_id] = True
                        RAD_TO_RAW = 4096.0 / (2.0 * math.pi)
                        if WRIST in self.config.motor_to_joint_dict[motor_id]:
                            avg_limit = float(np.mean(position_buffers[motor_id]))
                        else:
                            self.disable_torque([motor_id])
                            time.sleep(TINY_SLEEP)
                            _relaxed = self._motor_client.read_position_single(motor_id) if hasattr(self._motor_client, 'read_position_single') else None
                            avg_limit = float(_relaxed) if _relaxed is not None else float(np.mean(position_buffers[motor_id]))
                        avg_limit_raw = int(avg_limit * RAD_TO_RAW)
                        print(
                            f"Motor {motor_id} corresponding to joint {self.config.motor_to_joint_dict[motor_id]} reached the limit at raw {avg_limit_raw}."
                        )
                        if directions[motor_id] == 1:
                            motor_limits[motor_id][1] = avg_limit_raw
                        if directions[motor_id] == -1:
                            motor_limits[motor_id][0] = avg_limit_raw

                        if (
                            self._motor_client.requires_offset_calibration
                            and motor_id not in motors_with_final_offset
                        ):
                            is_positive = directions[motor_id] > 0
                            self._motor_client.calibrate_offset(
                                motor_id, upper=is_positive
                            )
                            time.sleep(TINY_SLEEP)
                            _new = self._motor_client.read_position_single(motor_id) if hasattr(self._motor_client, 'read_position_single') else None
                            new_limit = float(_new) if _new is not None else float(np.mean(position_buffers[motor_id]))
                            new_limit_raw = int(new_limit * RAD_TO_RAW)
                            motor_limits[motor_id][1 if is_positive else 0] = (
                                new_limit_raw
                            )
                            print(
                                f"  (Offset adjusted: limit now at raw {new_limit_raw})"
                            )
                            motors_with_final_offset.add(motor_id)

                        self.enable_torque([motor_id])

            for joint in step[JOINTS].keys():
                motor_id = self.config.joint_to_motor_map[joint]
                if (
                    motor_limits[motor_id][0] is None
                    or motor_limits[motor_id][1] is None
                ):
                    continue

                print("Joint calibrated: ", joint)
                calibrated_joints[joint] = 0.0

            # Persist partial progress after every step
            update_yaml(self.config.calibration_path, MOTOR_LIMITS_DICT, motor_limits)

            step_wrist_calibrated = self.calibration.wrist_calibrated or (
                WRIST in calibrated_joints
            )
            self.calibration = self._build_calibration_result(
                motor_limits=motor_limits,
                wrist_calibrated=step_wrist_calibrated,
            )
            update_yaml(
                self.config.calibration_path,
                WRIST_CALIBRATED,
                self.calibration.wrist_calibrated,
            )
            update_yaml(
                self.config.calibration_path,
                CALIBRATED,
                self.calibration.calibrated,
            )

            if calibrated_joints:
                self.set_joint_positions(
                    calibrated_joints, num_steps=NUM_STEPS, step_size=STEP_SIZE
                )

            # TODO(fracapuano): Is this necessary?
            time.sleep(0.1)

        new_wrist_calibrated = self.calibration.wrist_calibrated
        if any(WRIST in step[JOINTS] for step in calibration_sequence):
            new_wrist_calibrated = True
            update_yaml(self.config.calibration_path, WRIST_CALIBRATED, True)

        final_result = self._build_calibration_result(
            motor_limits=motor_limits,
            wrist_calibrated=new_wrist_calibrated,
        )
        self.calibration = final_result
        update_yaml(self.config.calibration_path, CALIBRATED, final_result.calibrated)

        if calibrated_joints:
            self.set_joint_positions(
                calibrated_joints, num_steps=NUM_STEPS, step_size=TINY_SLEEP
            )

        self.set_max_current(self.config.max_current)

        return final_result

    def set_neutral_position(self, num_steps: int = 1, step_size: float = 0.0):
        """Move hand to neutral position with overcurrent protection suspended."""
        control_mode = self.config.control_mode
        self.set_control_mode(POSITION)
        self._compute_wrap_offsets_dict()  # Always fresh before safety-critical move
        self._wrap_offsets_ts = time.time()
        with self.overcurrent.suspend(grace_sec=1.2):
            self._fast_move_to_joints(
                OrcaJointPositions.from_dict(self.config.neutral_position)
            )
        self.set_control_mode(control_mode)

    def set_zero_position(self, num_steps: int = 1, step_size: float = 0.0):
        """Move hand to zero with overcurrent protection suspended."""
        control_mode = self.config.control_mode
        self.set_control_mode(POSITION)
        self._compute_wrap_offsets_dict()
        self._wrap_offsets_ts = time.time()
        with self.overcurrent.suspend(grace_sec=1.2):
            self._fast_move_to_joints(
                OrcaJointPositions.from_dict(
                    {joint: 0.0 for joint in self.config.joint_ids}
                )
            )
        self.set_control_mode(control_mode)

    def _fast_move_to_joints(self, joint_pos: OrcaJointPositions) -> None:
        """Move all motors to target joint positions with max speed in one shot.

        Skips interpolation — writes target positions directly via sync write
        at maximum speed. Used for zeroing and neutral returns where speed
        and minimal power draw are critical.
        """
        joint_pos = self.config.clamp_joint_positions(joint_pos)
        target_motor_pos = self._joint_to_motor_pos(joint_pos.as_dict())

        ids, positions = [], []
        for idx, rad in enumerate(target_motor_pos):
            if rad is not None and not math.isnan(rad):
                ids.append(self.config.motor_ids[idx])
                positions.append(float(rad))

        if not ids:
            return

        with self._motor_lock:
            client = self._motor_client
            if client is None:
                return
            if hasattr(client, "write_positions_fast"):
                client.write_positions_fast(ids, np.array(positions, dtype=float))
            else:
                client.write_desired_pos(ids, np.array(positions, dtype=float))

    def _configure_overcurrent_from_config(self, limit_ma: float | None = None) -> None:
        """Configure overcurrent protection from hand config or explicit limit.

        Uses config.max_current as default limit if available.
        """
        default_limit = limit_ma if limit_ma is not None else None
        if default_limit is None:
            try:
                default_limit = float(self.config.max_current)
            except Exception:
                default_limit = None

        self.overcurrent.configure(
            enabled=(default_limit is not None and default_limit > 0),
            default_limit_ma=default_limit,
            latch=True,
            reset_ratio=0.80,
        )

    def _compute_wrap_offsets_dict(self):
        """Detect per-motor encoder wrap-arounds and store correction offsets.

        Handles any number of full turns by computing the offset from the
        center of the calibrated limit range. Skips motors whose position
        read failed (pos near 0 when limits are far from 0 → likely comm error).
        """
        TWO_PI = 2.0 * math.pi
        RAW_TO_RAD = TWO_PI / 4096.0
        motor_pos = self.get_motor_pos()

        offsets = {}
        for idx, motor_id in enumerate(self.config.motor_ids):
            limits = self.motor_limits_dict.get(motor_id)
            if limits is None or limits[0] is None or limits[1] is None:
                offsets[motor_id] = 0.0
                continue

            lo_rad = float(limits[0]) * RAW_TO_RAD
            hi_rad = float(limits[1]) * RAW_TO_RAD
            center_rad = (lo_rad + hi_rad) / 2.0
            pos_rad = float(motor_pos[idx])

            # Skip if position read failed (NaN from comm error, or zero far from range)
            if math.isnan(pos_rad) or abs(pos_rad) < 0.001 or abs(pos_rad - center_rad) > TWO_PI * 3:
                print(
                    f"Motor ID {motor_id}: pos={pos_rad:.3f} rad (likely read error), "
                    f"keeping previous offset"
                )
                offsets[motor_id] = self._wrap_offsets_dict.get(motor_id, 0.0) if self._wrap_offsets_dict else 0.0
                continue

            # How many full turns from the center of the working range?
            turns = round((pos_rad - center_rad) / TWO_PI)
            offset = float(turns) * TWO_PI

            normalized = pos_rad - offset
            if normalized < lo_rad - 0.25 * math.pi or normalized > hi_rad + 0.25 * math.pi:
                print(
                    f"Motor ID {motor_id}: pos={pos_rad:.3f} rad, "
                    f"limits=[{lo_rad:.3f}, {hi_rad:.3f}], "
                    f"turns={turns}, normalized={normalized:.3f} rad"
                )

            offsets[motor_id] = offset

        print(f"Offsets (turns): {{{', '.join(f'{mid}: {off/TWO_PI:.1f}t' for mid, off in offsets.items())}}}")
        self._wrap_offsets_dict = offsets

    def _set_motor_pos(
        self, desired_pos: Union[dict, np.ndarray, list], rel_to_current: bool = False
    ):
        with self._motor_lock:
            if (
                rel_to_current
            ):  # TODO(fracapuano): split in two methods for delta-set or absolute-set
                current_positions = self.get_motor_pos()

            motor_ids_to_write = []
            positions_to_write = []

            if isinstance(desired_pos, dict):
                for motor_id, pos_val in desired_pos.items():
                    if motor_id not in self.config.motor_ids:
                        print(
                            f"Warning: Motor ID {motor_id} in desired_pos dict is not in self.config.motor_ids. Skipping."
                        )
                        continue
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    pos_to_write = float(pos_val)
                    if rel_to_current:
                        pos_to_write += current_positions[
                            self.config.motor_id_to_idx_dict[motor_id]
                        ]

                    motor_ids_to_write.append(motor_id)
                    positions_to_write.append(pos_to_write)

                if not motor_ids_to_write:
                    return
                positions_to_write = np.array(positions_to_write, dtype=float)

            elif isinstance(desired_pos, (np.ndarray, list)):
                if len(desired_pos) != len(self.config.motor_ids):
                    raise ValueError(
                        f"Length of desired_pos (list/ndarray) ({len(desired_pos)}) must match the number of configured motor_ids ({len(self.config.motor_ids)})."
                    )

                for idx, pos_val in enumerate(desired_pos):
                    if pos_val is None or math.isnan(pos_val):
                        continue

                    motor_ids_to_write.append(self.config.motor_ids[idx])
                    if rel_to_current:
                        positions_to_write.append(
                            float(pos_val) + current_positions[idx]
                        )
                    else:
                        positions_to_write.append(float(pos_val))

                if not motor_ids_to_write:
                    print(
                        "\033[93mWarning: All positions in desired_pos (list/array) were None. No motor commands sent.\033[0m"
                    )
                    return

                positions_to_write = np.array(positions_to_write, dtype=float)

            else:
                raise ValueError("desired_pos must be a dict, np.ndarray, or list.")

            # Overcurrent guard: check currents and filter tripped servos
            if len(motor_ids_to_write) >= 2:
                moves = list(zip(motor_ids_to_write, positions_to_write))
                moves = self.overcurrent.guard_moves(moves)
                if not moves:
                    return
                motor_ids_to_write = [m[0] for m in moves]
                positions_to_write = np.array([m[1] for m in moves], dtype=float)

            self._motor_client.write_desired_pos(motor_ids_to_write, positions_to_write)

    def _motor_to_joint_pos(self, motor_pos: np.ndarray) -> dict:
        """Convert motor positions (radians) to joint angles (degrees 0–360)."""
        if self._wrap_offsets_dict is None:
            self._compute_wrap_offsets_dict()

        joint_pos = {}
        for idx, rad in enumerate(motor_pos):
            motor_id = self.config.motor_ids[idx]
            joint_name = self.config.motor_to_joint_dict.get(motor_id)
            if joint_name is None:
                continue
            if math.isnan(float(rad)):
                continue  # Skip motors with failed reads

            rad = rad - self._wrap_offsets_dict.get(motor_id, 0.0)
            raw = (rad / (2.0 * math.pi)) * 4096.0

            if self.config.joint_inversion_dict.get(joint_name, False):
                deg = (4095.0 - raw) / 4095.0 * 360.0
            else:
                deg = raw / 4095.0 * 360.0

            joint_pos[joint_name] = deg
        return joint_pos

    def _joint_to_motor_pos(self, joint_pos: dict) -> np.ndarray:
        """Convert joint angles (degrees 0–360) to motor positions (radians)."""
        # Recompute if never computed or stale (>2s old to avoid drift after mode switches)
        now = time.time()
        if self._wrap_offsets_dict is None or (now - getattr(self, '_wrap_offsets_ts', 0)) > 2.0:
            self._compute_wrap_offsets_dict()
            self._wrap_offsets_ts = now

        motor_pos = [None] * len(self.config.motor_ids)

        for joint_name, deg in joint_pos.items():
            motor_id = self.config.joint_to_motor_map.get(joint_name)
            if motor_id is None or deg is None:
                continue

            # deg (0–360) → raw (0–4095)
            inverted = self.config.joint_inversion_dict.get(joint_name, False)
            if inverted:
                raw = 4095.0 - (deg / 360.0) * 4095.0
            else:
                raw = (deg / 360.0) * 4095.0

            # clamp to mechanical limits
            limits = self.config.joint_limits_dict.get(joint_name)
            if limits:
                raw = max(limits[0], min(limits[1], raw))

            # raw → rad
            rad = raw * (2.0 * math.pi / 4096.0)

            idx = self.config.motor_id_to_idx_dict[motor_id]
            motor_pos[idx] = rad + self._wrap_offsets_dict.get(motor_id, 0.0)

        return motor_pos

    def _sanity_check(self):
        for motor_limit in self.motor_limits_dict.values():
            if any(limit is None for limit in motor_limit):
                self.calibration = dataclasses.replace(
                    self.calibration, calibrated=False
                )
                update_yaml(self.config.calibration_path, "calibrated", False)

    def tension(self, move_motors: bool = False, blocking: bool = True):
        """Hold motors under current to allow manual tendon tensioning.

        Optionally pre-conditions the tendons with a short back-and-forth
        motion before entering the hold phase. Torque is disabled automatically
        on exit.

        Args:
            move_motors: When ``True``, execute a short flexion/extension cycle
                before holding (default ``False``).
            blocking: When ``True`` (default) blocks until the user interrupts
                with Ctrl-C. When ``False`` runs in a background thread.
        """
        if blocking:
            self._task_stop_event.clear()
            self._tension(move_motors)
        else:
            self._start_task(self._tension, move_motors)

    def jitter(
        self,
        motor_ids: List[int] = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
        blocking: bool = True,
    ):
        """Apply a sinusoidal jitter to the motors for tendon seating.

        All motors oscillate around their current position with a sine wave.
        Amplitude is capped at 10° for safety.

        Args:
            motor_ids: Motors to jitter. Defaults to all non-wrist motors (or
                all motors when *include_wrist* is ``True``).
            amplitude: Peak-to-peak amplitude in degrees (default ``5.0``,
                max ``10.0``).
            frequency: Oscillation frequency in Hz (default ``10.0``).
            duration: Total jitter duration in seconds (default ``3.0``).
            include_wrist: Include the wrist motor when *motor_ids* is
                ``None`` (default ``False``).
            blocking: When ``True`` (default) blocks until jitter completes.
                When ``False`` runs in a background thread.

        Raises:
            ValueError: If *amplitude* exceeds 10°.
        """
        if blocking:
            self._task_stop_event.clear()
            self._jitter(motor_ids, amplitude, frequency, duration, include_wrist)
        else:
            self._start_task(
                self._jitter, motor_ids, amplitude, frequency, duration, include_wrist
            )

    def _jitter(
        self,
        motor_ids: List[int] = None,
        amplitude: float = 5.0,
        frequency: float = 10.0,
        duration: float = 3.0,
        include_wrist: bool = False,
    ):
        max_amplitude_deg = 10.0
        if amplitude > max_amplitude_deg:
            raise ValueError(
                f"Amplitude must be <= {max_amplitude_deg} degrees for safety. Got {amplitude}."
            )

        amplitude_rad = np.deg2rad(amplitude)

        if motor_ids is None:
            wrist_motor_id = self.config.joint_to_motor_map.get("wrist")
            motor_ids = [
                mid
                for mid in self.config.motor_ids
                if include_wrist or mid != wrist_motor_id
            ]

        start_positions = self.get_motor_pos(as_dict=True)
        start_pos_array = np.array([start_positions[mid] for mid in motor_ids])

        # Feetech (and similar) issue one bus transaction per motor per update.
        # Without a throttle, the inner loop floods the serial link and TxRx
        # fails ("no status packet" / "incorrect status packet").
        jitter_period_s = 0.01

        start_time = time.time()
        while (
            time.time() - start_time < duration and not self._task_stop_event.is_set()
        ):
            t = time.time() - start_time
            offset = amplitude_rad * math.sin(2 * math.pi * frequency * t)
            with self._motor_lock:
                self._motor_client.write_desired_pos(
                    motor_ids, start_pos_array + offset
                )
            time.sleep(jitter_period_s)

        with self._motor_lock:
            self._motor_client.write_desired_pos(motor_ids, start_pos_array)

    def _tension(self, move_motors: bool = False):
        # TODO(fracapuano): Move this to a standard stateless function
        control_mode = self.config.control_mode
        self.set_control_mode(CURRENT_BASED_POSITION)
        if move_motors:
            motors_to_move = [
                motor_id
                for joint, motor_id in self.config.joint_to_motor_map.items()
                if WRIST not in joint.lower() and motor_id in self.config.motor_ids
            ]
            self.set_max_current(self.config.calibration_current)

            duration = 8
            increment_per_step = 0.1
            motor_increments_right = {
                motor_id: increment_per_step for motor_id in motors_to_move
            }
            motor_increments_left = {
                motor_id: -increment_per_step for motor_id in motors_to_move
            }

            start_time = time.time()
            while time.time() - start_time < duration:
                if self._task_stop_event.is_set():
                    break
                self._set_motor_pos(motor_increments_left, rel_to_current=True)
                time.sleep(0.1)

            start_time = time.time()
            while time.time() - start_time < duration:
                if self._task_stop_event.is_set():
                    break
                self._set_motor_pos(motor_increments_right, rel_to_current=True)
                time.sleep(0.1)

        self.set_max_current(self.config.max_current)
        self.enable_torque()
        print("Holding motors. Please tension carefully. Press Ctrl+C to exit.")
        try:
            while not self._task_stop_event.is_set():
                time.sleep(0.1)
        finally:
            self.set_control_mode(control_mode)
            self.disable_torque()

    def _run_task(self, task_fn, *args, **kwargs):
        with self._lock:
            self._task_stop_event.clear()
            self._current_task = task_fn.__name__
            try:
                task_fn(*args, **kwargs)
            finally:
                self._current_task = None

    def _start_task(self, task_fn, *args, **kwargs):
        if self._task_thread and self._task_thread.is_alive():
            print(f"Task '{self._current_task}' is already running.")
            return

        self._task_thread = threading.Thread(
            target=self._run_task, args=(task_fn,) + args, kwargs=kwargs
        )
        self._task_thread.start()

    def stop_task(self):
        """Stops a background task like calibration, tensioning or jittering."""
        if self._task_thread and self._task_thread.is_alive():
            self._task_stop_event.set()
            self._task_thread.join()
            print("Task stopped.")
        else:
            print("No running task to stop.")


class MockOrcaHand(OrcaHand):
    """Drop-in :class:`OrcaHand` backed by an in-memory mock motor client,
    for testing and prototyping.

    All methods behave identically to :class:`OrcaHand` but no serial
    port is opened and motor state is simulated in memory.
    """

    def _create_motor_client(self) -> MotorClient:
        from .hardware.mock_dynamixel_client import MockDynamixelClient

        return MockDynamixelClient(
            self.config.motor_ids, self.config.port, self.config.baudrate
        )
