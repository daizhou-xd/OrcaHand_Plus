"""Overcurrent protection for ORCA Hand Feetech servos.

Monitors servo current draw via sync-read and trips torque-off when any
servo exceeds its configured limit. Ported from orca-feetech-STS3215
reference implementation and adapted for my_orca's OrcaHand architecture.

Key features:
- Per-servo current limits (mA)
- Spike filtering: physical max, jump delta, confirm-hits
- Latch / auto-reset modes
- Grace period after reset
- Suspend during critical operations (zeroing, e-stop)
- Background monitoring thread with throttled polling
"""

import contextlib
import logging
import threading
import time
from typing import Optional

logger = logging.getLogger("orca_core.overcurrent")


class OvercurrentProtection:
    """Per-hand overcurrent monitor and protector.

    Usage inside OrcaHand:
        oc = OvercurrentProtection(hand)
        oc.configure(enabled=True, limit_ma=800)
        # After connect:
        oc.start_monitor()
        # Before disconnect:
        oc.stop_monitor()
    """

    def __init__(self, hand):
        """*hand* must be an OrcaHand instance (needs motor_client, motor_ids, etc.)."""
        self._hand = hand

        # ---- config ----
        self._enabled: bool = False
        self._default_limit_ma: float | None = None
        self._limit_ma_by_id: dict[int, float] = {}
        self._latch: bool = True
        self._reset_ratio: float = 0.80

        # ---- filter params ----
        self._phys_max_ma: float = 3250.0
        self._jump_delta_ma: float = 1500.0
        self._confirm_hits: int = 2  # require N consecutive over-limit samples

        # ---- runtime state ----
        self._tripped: set[int] = set()
        self._trip_log_once: set[int] = set()
        self._last_ok_ma: dict[int, float] = {}
        self._over_limit_hits: dict[int, int] = {}
        self._last_ma: dict[int, float] = {}

        # ---- suspension / grace ----
        self._suspend_lock = threading.Lock()
        self._suspend_count: int = 0
        self._grace_until_ts: float = 0.0

        # ---- monitor thread ----
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_stop = threading.Event()
        self._monitor_interval: float = 0.08
        self._last_poll_ts: float = 0.0
        self._poll_min_interval: float = 0.06

    # ---- configuration -------------------------------------------------

    def configure(
        self,
        *,
        enabled: bool = False,
        default_limit_ma: float | None = None,
        limit_ma_by_id: dict[int, float] | None = None,
        latch: bool = True,
        reset_ratio: float = 0.80,
    ) -> None:
        self._enabled = bool(enabled)
        self._default_limit_ma = None if default_limit_ma is None else float(default_limit_ma)
        self._latch = bool(latch)
        try:
            rr = float(reset_ratio)
            self._reset_ratio = rr if 0.0 < rr < 1.0 else 0.80
        except Exception:
            self._reset_ratio = 0.80

        parsed: dict[int, float] = {}
        if isinstance(limit_ma_by_id, dict):
            for k, v in limit_ma_by_id.items():
                try:
                    sid, ma = int(k), float(v)
                    if 0 <= sid <= 253 and ma > 0:
                        parsed[int(sid)] = float(ma)
                except Exception:
                    continue
        self._limit_ma_by_id = parsed

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def any_limit_configured(self) -> bool:
        return bool(self._limit_ma_by_id) or (self._default_limit_ma is not None)

    @property
    def tripped_ids(self) -> set[int]:
        return set(self._tripped)

    @property
    def last_currents(self) -> dict[int, float]:
        return dict(self._last_ma)

    def effective_limit_ma(self, sid: int) -> float | None:
        v = self._limit_ma_by_id.get(int(sid))
        if v is not None:
            return float(v)
        return self._default_limit_ma

    # ---- suspension / grace --------------------------------------------

    def is_suspended(self) -> bool:
        try:
            with self._suspend_lock:
                return int(self._suspend_count) > 0
        except Exception:
            return False

    def in_grace_period(self) -> bool:
        try:
            return float(time.monotonic()) < float(self._grace_until_ts)
        except Exception:
            return False

    @contextlib.contextmanager
    def suspend(self, *, grace_sec: float = 0.0):
        """Temporarily suspend overcurrent trips for critical operations."""
        try:
            with self._suspend_lock:
                self._suspend_count = int(self._suspend_count) + 1
        except Exception:
            pass
        try:
            yield
        finally:
            try:
                if float(grace_sec) > 0:
                    self._grace_until_ts = max(
                        float(self._grace_until_ts),
                        float(time.monotonic()) + float(grace_sec),
                    )
            except Exception:
                pass
            try:
                with self._suspend_lock:
                    self._suspend_count = max(0, int(self._suspend_count) - 1)
            except Exception:
                pass

    # ---- filter --------------------------------------------------------

    def _filter_sample(self, sid: int, ma_abs: float) -> float | None:
        """Filter implausible current spikes. Returns filtered mA or None."""
        try:
            v = abs(float(ma_abs))
        except Exception:
            return None
        if v <= 0.0:
            return None
        if v > float(self._phys_max_ma):
            return None

        prev = self._last_ok_ma.get(int(sid))
        if prev is not None:
            try:
                if (float(v) - float(prev)) > float(self._jump_delta_ma):
                    return None
            except Exception:
                pass
        self._last_ok_ma[int(sid)] = float(v)
        return float(v)

    # ---- trip / reset --------------------------------------------------

    def _trip_servo(self, sid: int, ma: float, limit: float) -> bool:
        """Trip a servo if it exceeds the limit. Returns True if newly tripped."""
        self._over_limit_hits[int(sid)] = int(self._over_limit_hits.get(int(sid), 0)) + 1
        if int(self._over_limit_hits[int(sid)]) < int(self._confirm_hits):
            return False

        if int(sid) in self._tripped:
            return False

        self._tripped.add(int(sid))
        if int(sid) not in self._trip_log_once:
            self._trip_log_once.add(int(sid))
            logger.warning(
                "Over-current trip: sid=%s current=%.1fmA >= limit=%.1fmA (latch=%s)",
                int(sid), float(ma), float(limit), bool(self._latch),
            )
        return True

    def _maybe_reset_servo(self, sid: int, ma: float, limit: float) -> bool:
        """Auto-reset a tripped servo if current drops below reset threshold."""
        if self._latch:
            return False
        if int(sid) not in self._tripped:
            return False
        reset_threshold = float(limit) * float(self._reset_ratio)
        if float(ma) <= reset_threshold:
            self._tripped.discard(int(sid))
            self._trip_log_once.discard(int(sid))
            self._over_limit_hits[int(sid)] = 0
            logger.info(
                "Over-current auto-reset: sid=%s current=%.1fmA <= reset=%.1fmA",
                int(sid), float(ma), float(reset_threshold),
            )
            # Re-enable torque
            self._enable_torque_for([int(sid)])
            return True
        return False

    def reset_trips(self, servo_ids: list[int] | None = None) -> None:
        """Reset tripped state and re-enable torque."""
        try:
            self._grace_until_ts = float(time.monotonic()) + 0.8
        except Exception:
            pass

        if servo_ids is None:
            ids = list(self._tripped)
            self._tripped.clear()
            self._trip_log_once.clear()
            self._last_ok_ma.clear()
            self._over_limit_hits.clear()
        else:
            s = {int(x) for x in servo_ids}
            self._tripped.difference_update(s)
            self._trip_log_once.difference_update(s)
            for sid in list(s):
                self._last_ok_ma.pop(int(sid), None)
                self._over_limit_hits.pop(int(sid), None)
            ids = sorted(s)

        if ids:
            self._enable_torque_for(ids)

    # ---- internal hardware helpers -------------------------------------

    def _enable_torque_for(self, servo_ids: list[int]) -> None:
        """Best-effort torque re-enable for tripped servos."""
        hand = self._hand
        if hand is None or hand._motor_client is None:
            return
        try:
            hand._motor_client.set_torque_enabled(servo_ids, True)
        except Exception:
            pass

    def _disable_torque_for(self, servo_ids: list[int]) -> None:
        """Best-effort torque disable for tripped servos."""
        hand = self._hand
        if hand is None or hand._motor_client is None:
            return
        try:
            hand._motor_client.set_torque_enabled(servo_ids, False)
        except Exception:
            pass

    def _stop_tripped_motors(self, newly_tripped: list[int]) -> None:
        """Disable torque and set goal position to current position for tripped motors."""
        hand = self._hand
        if hand is None or hand._motor_client is None:
            return

        self._disable_torque_for(newly_tripped)

        # Set goal to current position so the servo stays in place
        try:
            for sid in newly_tripped:
                pos = hand._motor_client.read_position_single(sid)
                if pos is not None:
                    hand._motor_client.write_desired_pos([sid], [pos])
                    time.sleep(0.001)
        except Exception:
            pass

    # ---- current polling -----------------------------------------------

    def poll_currents(self, motor_ids: list[int]) -> dict[int, float]:
        """Read currents for *motor_ids* and process overcurrent logic.

        Returns newly tripped servo IDs.
        """
        hand = self._hand
        if hand is None or hand._motor_client is None:
            return {}

        # Throttle polling
        now = time.time()
        if (now - float(self._last_poll_ts)) < float(self._poll_min_interval):
            return {}
        self._last_poll_ts = float(now)

        client = hand._motor_client
        if not hasattr(client, "sync_read_currents"):
            return {}

        currents = client.sync_read_currents(list(motor_ids))
        if not isinstance(currents, dict):
            return {}

        newly_tripped: list[int] = []
        for sid in motor_ids:
            if self.is_suspended() or self.in_grace_period():
                break

            ma = currents.get(int(sid))
            if ma is None:
                continue

            ma_f = self._filter_sample(int(sid), abs(float(ma)))
            if ma_f is None:
                continue
            self._last_ma[int(sid)] = float(ma_f)

            limit = self.effective_limit_ma(int(sid))
            if limit is None:
                continue

            if float(ma_f) >= float(limit):
                if self._trip_servo(int(sid), float(ma_f), float(limit)):
                    newly_tripped.append(int(sid))
            else:
                self._over_limit_hits[int(sid)] = 0
                self._maybe_reset_servo(int(sid), float(ma_f), float(limit))

        if newly_tripped:
            self._stop_tripped_motors(newly_tripped)

        return currents

    # ---- background monitor --------------------------------------------

    def start_monitor(self) -> None:
        """Start background overcurrent monitoring thread."""
        if self._monitor_thread is not None and self._monitor_thread.is_alive():
            return

        self._monitor_stop.clear()
        hand = self._hand

        def _loop():
            while not self._monitor_stop.is_set():
                try:
                    if self.is_suspended() or self.in_grace_period():
                        time.sleep(0.05)
                        continue

                    if (hand is None or hand._motor_client is None
                            or not self._enabled or not self.any_limit_configured
                            or not hasattr(hand._motor_client, "sync_read_currents")):
                        time.sleep(0.10)
                        continue

                    # Poll all motor IDs
                    motor_ids = list(hand.config.motor_ids)
                    if not motor_ids:
                        time.sleep(0.10)
                        continue

                    self.poll_currents(motor_ids)
                    time.sleep(float(self._monitor_interval))

                except Exception:
                    time.sleep(0.10)

        self._monitor_thread = threading.Thread(target=_loop, daemon=True)
        self._monitor_thread.start()

    def stop_monitor(self) -> None:
        """Stop the background monitor thread."""
        self._monitor_stop.set()
        self._monitor_thread = None

    # ---- control-path guard (called before every position write) --------

    def guard_moves(
        self,
        moves: list[tuple[int, float]],
    ) -> list[tuple[int, float]]:
        """Check overcurrent before a position write; filter out tripped servos.

        Called from the control path before sending position commands.
        Runs a current poll if enough time has passed since the last one.
        Returns filtered move list with tripped servos removed.
        """
        if self.is_suspended() or self.in_grace_period():
            return list(moves)

        if not self._enabled or not self.any_limit_configured:
            return list(moves)

        hand = self._hand
        if hand is None or hand._motor_client is None:
            return list(moves)

        if not hasattr(hand._motor_client, "sync_read_currents"):
            return list(moves)

        # Throttled poll
        ids_in_moves = sorted({int(sid) for sid, _pos in moves if 0 <= int(sid) <= 253})
        if ids_in_moves:
            self.poll_currents(ids_in_moves)

        # Filter out tripped servos
        if self._tripped:
            return [(sid, pos) for (sid, pos) in moves if int(sid) not in self._tripped]

        return list(moves)
