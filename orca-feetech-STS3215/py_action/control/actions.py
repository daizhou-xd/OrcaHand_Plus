import os
import sys
import tempfile
import time
import threading
import logging
import contextlib
import math


def _ocra_temp_dir() -> str:
    return os.path.join(tempfile.gettempdir(), "ocra_py_action")


def _configure_pycache_prefix() -> None:
    try:
        base = os.environ.get("OCRA_PYCACHE_DIR") or os.path.join(_ocra_temp_dir(), "pycache")
        os.makedirs(base, exist_ok=True)
        sys.pycache_prefix = base  # type: ignore[attr-defined]
    except Exception:
        pass


def _bootstrap_module_search_path() -> None:
    control_dir = os.path.abspath(os.path.dirname(__file__))
    py_action_root = os.path.abspath(os.path.join(control_dir, os.pardir))
    workspace_root = os.path.abspath(os.path.join(py_action_root, os.pardir))

    paths = [
        workspace_root,
        py_action_root,
        os.path.join(py_action_root, "hardware"),
        control_dir,
    ]
    for p in reversed(paths):
        if p not in sys.path:
            sys.path.insert(0, p)

_configure_pycache_prefix()
_bootstrap_module_search_path()

from servo_bus import ServoBus


logger = logging.getLogger("ocra_py_action.control.actions")

# ======================
# 全局资源
# ======================
bus: ServoBus | None = None
bus_lock = threading.Lock()

stop_event = threading.Event()
_action_thread = None

# Used to abort long-running calibration workflows (e.g. Action 7 in GUI).
_calibration_abort_event = threading.Event()

# 暂停：只用于“预设动作/序列”调试标定，不影响手动滑块写入
pause_event = threading.Event()

# 写入参数（供 GUI 调整）
_write_speed = 1000

# ======================
# 过流保护（按舵机电流寄存器反馈）
# ======================
_oc_enabled: bool = False
_oc_limit_ma: float | None = None  # default limit (back-compat)
_oc_limit_ma_by_id: dict[int, float] = {}
_oc_latch: bool = True
_oc_reset_ratio: float = 0.80
_oc_tripped: set[int] = set()
_oc_last_ma: dict[int, float] = {}
_oc_trip_log_once: set[int] = set()
_oc_last_poll_ts: float = 0.0
_oc_poll_min_interval_sec: float = 0.06

# Over-current sampling robustness (avoid single-sample spikes causing false trips)
_oc_filter_phys_max_ma: float = 3250.0
_oc_filter_jump_delta_ma: float = 1500.0
_oc_trip_confirm_hits: int = 2  # require N consecutive samples over limit to trip
_oc_filter_last_ok_ma: dict[int, float] = {}
_oc_over_limit_hits: dict[int, int] = {}

_oc_monitor_thread: threading.Thread | None = None
_oc_monitor_stop_event = threading.Event()
_oc_monitor_interval_sec: float = 0.08
_oc_monitor_ids: list[int] = list(range(1, 18))

# 过流保护暂停/宽限：用于“回零/急停”等必须执行的动作，避免限流拦截。
_oc_suspend_lock = threading.Lock()
_oc_suspend_count: int = 0
_oc_grace_until_ts: float = 0.0


def _oc_is_suspended() -> bool:
    try:
        with _oc_suspend_lock:
            return int(_oc_suspend_count) > 0
    except Exception:
        return False


def _oc_in_grace_period() -> bool:
    try:
        return float(time.monotonic()) < float(_oc_grace_until_ts)
    except Exception:
        return False


@contextlib.contextmanager
def suspend_overcurrent_protection(*, grace_sec: float = 0.0):
    """Temporarily suspend overcurrent trips (monitor + control-path) for critical operations.

    Optionally extends a short grace period after the suspension.
    """
    global _oc_suspend_count, _oc_grace_until_ts
    try:
        with _oc_suspend_lock:
            _oc_suspend_count = int(_oc_suspend_count) + 1
    except Exception:
        pass
    try:
        yield
    finally:
        try:
            if float(grace_sec) > 0:
                _oc_grace_until_ts = max(float(_oc_grace_until_ts), float(time.monotonic()) + float(grace_sec))
        except Exception:
            pass
        try:
            with _oc_suspend_lock:
                _oc_suspend_count = max(0, int(_oc_suspend_count) - 1)
        except Exception:
            pass


def _best_effort_enable_torque(
    servo_ids: list[int],
    *,
    lock_timeout: float = 0.15,
    retries: int = 0,
    retry_sleep_sec: float = 0.05,
) -> bool:
    """Try to re-enable torque for ids. Returns True if command was sent.

    Uses SYNCWRITE when available, falls back to per-servo write_u8.
    """
    if bus is None:
        return False
    ids = [int(x) for x in servo_ids if 0 <= int(x) <= 253]
    if not ids:
        return False

    for _ in range(max(1, int(retries) + 1)):
        got = bus_lock.acquire(timeout=float(lock_timeout))
        if not got:
            time.sleep(float(retry_sleep_sec))
            continue
        try:
            _require_bus()
            addr = int(getattr(bus, "TORQUE_ENABLE_ADDR", 0x28))
            payload = bytes([0x01])
            if hasattr(bus, "sync_write"):
                bus.sync_write([(int(sid), payload) for sid in ids], start_addr=addr, data_len=1)
                return True
            # Fallback
            if hasattr(bus, "write_u8"):
                for sid in ids:
                    try:
                        bus.write_u8(int(sid), int(addr), 1)
                    except Exception:
                        pass
                return True
            return False
        except Exception:
            time.sleep(float(retry_sleep_sec))
            continue
        finally:
            try:
                bus_lock.release()
            except Exception:
                pass
    return False


def configure_overcurrent_protection(
    *,
    enabled: bool,
    limit_ma: float | None,
    limit_ma_by_id: dict[int, float] | None = None,
    latch: bool = True,
    reset_ratio: float = 0.80,
) -> None:
    """配置过流保护。

    - enabled=False：完全关闭
    - limit_ma：阈值（mA）；None 表示关闭
    - latch=True：触发后保持熔断，直到手动 reset 或断开重连
    - latch=False：允许电流降到 limit_ma*reset_ratio 后自动恢复
    """
    global _oc_enabled, _oc_limit_ma, _oc_limit_ma_by_id, _oc_latch, _oc_reset_ratio

    parsed_by_id: dict[int, float] = {}
    if isinstance(limit_ma_by_id, dict):
        for k, v in limit_ma_by_id.items():
            try:
                sid = int(k)
                ma = float(v)
                if 0 <= sid <= 253 and ma > 0:
                    parsed_by_id[int(sid)] = float(ma)
            except Exception:
                continue

    _oc_limit_ma_by_id = parsed_by_id
    _oc_limit_ma = None if limit_ma is None else float(limit_ma)
    _oc_enabled = bool(enabled) and (bool(_oc_limit_ma_by_id) or (_oc_limit_ma is not None))
    _oc_latch = bool(latch)
    try:
        rr = float(reset_ratio)
        _oc_reset_ratio = rr if 0.0 < rr < 1.0 else 0.80
    except Exception:
        _oc_reset_ratio = 0.80


def _oc_effective_limit_ma(sid: int) -> float | None:
    try:
        sid_i = int(sid)
    except Exception:
        return _oc_limit_ma
    v = _oc_limit_ma_by_id.get(int(sid_i))
    if v is not None:
        return float(v)
    return _oc_limit_ma


def _oc_any_limit_configured() -> bool:
    return bool(_oc_limit_ma_by_id) or (_oc_limit_ma is not None)


def reset_overcurrent_trips(servo_ids: list[int] | None = None) -> None:
    global _oc_tripped, _oc_trip_log_once
    global _oc_grace_until_ts
    global _oc_filter_last_ok_ma, _oc_over_limit_hits
    # Give a short grace window after reset so we don't immediately re-trip on stale reads.
    try:
        _oc_grace_until_ts = float(time.monotonic()) + 0.8
    except Exception:
        pass
    if servo_ids is None:
        _oc_tripped.clear()
        _oc_trip_log_once.clear()
        _oc_filter_last_ok_ma.clear()
        _oc_over_limit_hits.clear()
        # Best-effort: re-enable torque for all managed servos (retry in background if busy).
        ok = False
        try:
            ok = _best_effort_enable_torque(list(_oc_monitor_ids), lock_timeout=0.08, retries=1)
        except Exception:
            ok = False
        if not ok:
            def _retry():
                try:
                    _best_effort_enable_torque(list(_oc_monitor_ids), lock_timeout=0.20, retries=6, retry_sleep_sec=0.06)
                except Exception:
                    pass

            threading.Thread(target=_retry, daemon=True).start()
        return
    s = {int(x) for x in servo_ids}
    _oc_tripped.difference_update(s)
    _oc_trip_log_once.difference_update(s)
    for sid in list(s):
        try:
            _oc_filter_last_ok_ma.pop(int(sid), None)
            _oc_over_limit_hits.pop(int(sid), None)
        except Exception:
            pass
    # Best-effort: re-enable torque for selected ids.
    ok = False
    try:
        ok = _best_effort_enable_torque(sorted(s), lock_timeout=0.08, retries=1)
    except Exception:
        ok = False
    if not ok:
        def _retry_sel():
            try:
                _best_effort_enable_torque(sorted(s), lock_timeout=0.20, retries=6, retry_sleep_sec=0.06)
            except Exception:
                pass

        threading.Thread(target=_retry_sel, daemon=True).start()


def get_overcurrent_status() -> dict:
    return {
        "enabled": bool(_oc_enabled),
        "limit_ma": _oc_limit_ma,
        "limit_ma_by_id": dict(_oc_limit_ma_by_id),
        "latch": bool(_oc_latch),
        "reset_ratio": float(_oc_reset_ratio),
        "tripped_ids": sorted(int(x) for x in _oc_tripped),
    }


def get_last_currents_ma(servo_ids: list[int] | None = None) -> dict[int, float]:
    """Return last observed current (abs mA) snapshot.

    Values are updated by overcurrent monitor / control-path polling when enabled.
    """
    if servo_ids is None:
        return dict(_oc_last_ma)
    out: dict[int, float] = {}
    for sid in servo_ids:
        try:
            v = _oc_last_ma.get(int(sid))
            if v is not None:
                out[int(sid)] = float(v)
        except Exception:
            continue
    return out


def _read_currents_best_effort(
    servo_ids: list[int],
    *,
    lock_timeout: float = 0.01,
    max_wait: float = 0.06,
) -> dict[int, float]:
    """Best-effort current sampling without blocking control writes.

    Returns a dict sid->abs(mA) for values that are readable.
    """
    if bus is None or (not hasattr(bus, "sync_read_current_ma")):
        return {}

    got = bus_lock.acquire(timeout=float(lock_timeout))
    if not got:
        return {}
    try:
        _require_bus()
        raw = bus.sync_read_current_ma(list(servo_ids), max_wait=float(max_wait))  # type: ignore[attr-defined]
        out: dict[int, float] = {}
        if isinstance(raw, dict):
            for sid, v in raw.items():
                if v is None:
                    continue
                try:
                    out[int(sid)] = abs(float(v))
                except Exception:
                    continue
        return out
    finally:
        try:
            bus_lock.release()
        except Exception:
            pass


def calibrate_overcurrent_limits_from_actions(
    action_funcs: list,
    *,
    servo_ids: list[int] | None = None,
    margin_ma: float = 10.0,
    sample_interval_sec: float = 0.05,
    lock_timeout_sec: float = 0.01,
    status_cb=None,
    final_zero: bool = True,
) -> dict[int, float]:
    """Run actions and measure per-servo max current, then return (max+margin).

    This is intended for GUI-side 'auto calibrate current' and writes are not
    performed here.
    """
    if bus is None:
        raise RuntimeError("not connected")

    # Start a new calibration session.
    try:
        _calibration_abort_event.clear()
    except Exception:
        pass

    # Temporarily enable current polling with a very high limit so we won't trip,
    # but _oc_last_ma keeps updating during actions.
    prev_oc: dict = {}
    try:
        prev_oc = get_overcurrent_status()
    except Exception:
        prev_oc = {}

    try:
        reset_overcurrent_trips(None)
        _oc_last_ma.clear()
        _oc_trip_log_once.clear()
        _oc_tripped.clear()
    except Exception:
        pass

    try:
        configure_overcurrent_protection(enabled=True, limit_ma=10000.0, limit_ma_by_id=None, latch=True, reset_ratio=0.80)
    except Exception:
        pass

    ids = list(range(1, 18)) if servo_ids is None else [int(x) for x in servo_ids]
    ids = [int(x) for x in ids if 0 <= int(x) <= 253]
    if not ids:
        return {}

    max_by_id: dict[int, float] = {}

    try:
        for idx, fn in enumerate(list(action_funcs)):
            if _calibration_abort_event.is_set() or stop_event.is_set():
                raise RuntimeError("calibration aborted")
            if callable(status_cb):
                try:
                    status_cb(f"🟣 电流校准：动作 {idx + 1}/{len(action_funcs)}")
                except Exception:
                    pass

            # Run action with implicit auto-zero so each action starts from a safe neutral pose.
            run_action(fn, status_cb, auto_zero=True)

            # Sample while action thread is alive.
            start_ts = time.monotonic()
            empty_streak = 0
            while is_action_running():
                if _calibration_abort_event.is_set():
                    break
                if stop_event.is_set():
                    try:
                        _calibration_abort_event.set()
                    except Exception:
                        pass
                    break

                cur = get_last_currents_ma(ids)

                # Always try a best-effort direct read (won't block if lock is busy).
                direct = _read_currents_best_effort(ids, lock_timeout=float(lock_timeout_sec), max_wait=0.10)
                if direct:
                    # Merge (direct wins)
                    cur = {**(cur or {}), **direct}

                if not cur:
                    empty_streak += 1
                else:
                    empty_streak = 0

                for sid, ma in (cur or {}).items():
                    prev = max_by_id.get(int(sid), 0.0)
                    if float(ma) > float(prev):
                        max_by_id[int(sid)] = float(ma)

                time.sleep(max(0.01, float(sample_interval_sec)))

                # Safety: don't spin forever if something goes wrong.
                if (time.monotonic() - start_ts) > 45.0:
                    break

            time.sleep(0.10)

            if _calibration_abort_event.is_set() or stop_event.is_set():
                raise RuntimeError("calibration aborted")

        # Return to zero at the end (but not part of measurement).
        if final_zero and (not stop_event.is_set()) and (not _calibration_abort_event.is_set()):
            try:
                if callable(status_cb):
                    status_cb("🟡 电流校准：回零")
                action_all_zero()
            except Exception:
                pass

        # Apply margin and filter out never-observed servos.
        out: dict[int, float] = {}
        for sid, mx in max_by_id.items():
            try:
                v = float(mx) + float(margin_ma)
                if v > 0:
                    out[int(sid)] = float(v)
            except Exception:
                continue
        return out
    finally:
        # Restore previous OC settings.
        try:
            configure_overcurrent_protection(
                enabled=bool(prev_oc.get("enabled", False)),
                limit_ma=prev_oc.get("limit_ma", None),
                limit_ma_by_id=dict(prev_oc.get("limit_ma_by_id", {}) or {}),
                latch=bool(prev_oc.get("latch", True)),
                reset_ratio=float(prev_oc.get("reset_ratio", 0.80)),
            )
        except Exception:
            pass


def calibrate_overcurrent_max_currents_fist_repeats(
    *,
    repeats: int = 5,
    servo_ids: list[int] | None = None,
    sample_interval_sec: float = 0.05,
    lock_timeout_sec: float = 0.01,
    rest_base_sec: float = 1.2,
    max_action_base_sec: float = 12.0,
    # Conservative filtering / robust peak estimation
    phys_max_ma: float = 3250.0,
    jump_delta_ma: float = 1500.0,
    near_ratio: float = 0.90,
    confirm_hits: int = 3,
    quantile: float = 0.98,
    status_cb=None,
) -> dict[int, float]:
    """Measure per-servo max current during 'Action 2: fist' repeated N times.

        The max is sampled ONLY while action_2_fist() is executing (zeroing/rest is excluded).

        To avoid communication glitches / single-sample spikes, this uses a conservative
        robust estimator:
        - Discard samples above phys_max_ma
        - Discard implausible jumps relative to last accepted sample (jump_delta_ma)
        - Discard samples above phys_max_ma
        - Discard implausible jumps relative to last accepted sample (jump_delta_ma)
        - Use a rolling median window (size=confirm_hits, default 3) to suppress one-off spikes
        - Final per-servo peak = P(quantile) over the median-filtered samples

        Returns raw max-by-id (abs mA). Caller can add margin and write YAML.
    """
    if bus is None:
        raise RuntimeError("not connected")

    try:
        n = int(repeats)
    except Exception:
        n = 5
    n = max(1, min(20, int(n)))

    # Start a new calibration session.
    try:
        _calibration_abort_event.clear()
    except Exception:
        pass

    prev_oc: dict = {}
    try:
        prev_oc = get_overcurrent_status()
    except Exception:
        prev_oc = {}

    # Temporarily enable current polling with a very high limit so we won't trip.
    try:
        reset_overcurrent_trips(None)
        _oc_last_ma.clear()
        _oc_trip_log_once.clear()
        _oc_tripped.clear()
    except Exception:
        pass

    try:
        configure_overcurrent_protection(enabled=True, limit_ma=10000.0, limit_ma_by_id=None, latch=True, reset_ratio=0.80)
    except Exception:
        pass

    ids = list(range(1, 18)) if servo_ids is None else [int(x) for x in servo_ids]
    ids = [int(x) for x in ids if 0 <= int(x) <= 253]
    if not ids:
        return {}

    try:
        q = float(quantile)
    except Exception:
        q = 0.98
    q = 0.98 if not (0.0 < q < 1.0) else q

    # N=3 is interpreted as a median filter window to suppress single-sample spikes.
    try:
        win = int(confirm_hits)
    except Exception:
        win = 3
    win = max(3, min(7, int(win)))

    # Keep near_ratio for backward compatibility (unused in the current robust estimator).
    try:
        _ = float(near_ratio)
    except Exception:
        pass

    try:
        pmax = float(phys_max_ma)
    except Exception:
        pmax = 3250.0
    pmax = max(500.0, float(pmax))

    try:
        jmax = float(jump_delta_ma)
    except Exception:
        jmax = 1500.0
    jmax = max(200.0, float(jmax))

    # Per-servo robust peak tracking state.
    last_ok: dict[int, float] = {}
    last_win: dict[int, list[float]] = {int(sid): [] for sid in ids}
    samples: dict[int, list[float]] = {int(sid): [] for sid in ids}

    # Timeouts/rest should respect user's speed->wait conversion.
    try:
        max_action_sec = max(15.0, float(_suggest_wait_seconds(float(max_action_base_sec))))
    except Exception:
        max_action_sec = 20.0

    try:
        for i in range(int(n)):
            if _calibration_abort_event.is_set() or stop_event.is_set():
                raise RuntimeError("calibration aborted")

            if callable(status_cb):
                try:
                    status_cb(f"🟣 电流校准：握拳 {i + 1}/{n}")
                except Exception:
                    pass

            # Execute the real action_2_fist() so any future edits (targets/steps) auto-sync here.
            done = threading.Event()
            err: list[Exception] = []

            def _run():
                try:
                    action_2_fist()
                except Exception as e:
                    err.append(e)
                finally:
                    done.set()

            t = threading.Thread(target=_run, daemon=True)
            t.start()

            # Sample ONLY while the action thread runs.
            start_ts = time.monotonic()
            while not done.is_set():
                if _calibration_abort_event.is_set() or stop_event.is_set():
                    raise RuntimeError("calibration aborted")

                cur = get_last_currents_ma(ids)
                direct = _read_currents_best_effort(ids, lock_timeout=float(lock_timeout_sec), max_wait=0.10)
                if direct:
                    cur = {**(cur or {}), **direct}

                # Conservative filtering + rolling median (ignore single-sample spikes)
                for sid in ids:
                    v0 = (cur or {}).get(int(sid))
                    if v0 is None:
                        continue
                    try:
                        v = abs(float(v0))
                    except Exception:
                        continue

                    # Physical bound
                    if (v <= 0.0) or (v > float(pmax)):
                        continue

                    # Jump bound (relative to last accepted value)
                    prev_ok = last_ok.get(int(sid))
                    if (prev_ok is not None) and ((float(v) - float(prev_ok)) > float(jmax)):
                        continue
                    last_ok[int(sid)] = float(v)

                    w = last_win.get(int(sid))
                    if w is None:
                        w = []
                        last_win[int(sid)] = w
                    w.append(float(v))
                    if len(w) > int(win):
                        del w[0 : (len(w) - int(win))]
                    if len(w) >= int(win):
                        try:
                            sw = sorted(w)
                            med = float(sw[len(sw) // 2])
                            samples[int(sid)].append(float(med))
                        except Exception:
                            pass

                time.sleep(max(0.01, float(sample_interval_sec)))

                # Safety: don't spin forever if something goes wrong.
                if (time.monotonic() - start_ts) > float(max_action_sec):
                    raise RuntimeError("calibration timeout")

            try:
                t.join(timeout=0.1)
            except Exception:
                pass

            if err:
                raise err[0]

            # Return to zero between fists (not part of sampling)
            try:
                action_all_zero()
            except Exception:
                pass
            try:
                # 间隔时间也用 speed->等待 的换算，避免速度调慢后还没到位就进入下一次。
                safe_sleep(_suggest_wait_seconds(float(rest_base_sec)))
            except Exception:
                pass

        # Final: per-servo P(quantile) over median-filtered samples.
        out: dict[int, float] = {}
        for sid in ids:
            vals = samples.get(int(sid)) or []
            if not vals:
                continue
            try:
                svals = sorted(float(x) for x in vals)
            except Exception:
                continue
            if not svals:
                continue
            # Use ceil so small sample sizes still pick the upper tail.
            k = int(math.ceil(float(q) * float(len(svals) - 1)))
            k = max(0, min(len(svals) - 1, int(k)))
            try:
                out[int(sid)] = float(svals[int(k)])
            except Exception:
                continue
        return out
    finally:
        try:
            configure_overcurrent_protection(
                enabled=bool(prev_oc.get("enabled", False)),
                limit_ma=prev_oc.get("limit_ma", None),
                limit_ma_by_id=dict(prev_oc.get("limit_ma_by_id", {}) or {}),
                latch=bool(prev_oc.get("latch", True)),
                reset_ratio=float(prev_oc.get("reset_ratio", 0.80)),
            )
        except Exception:
            pass



def is_connected():
    return bus is not None


def is_paused() -> bool:
    return pause_event.is_set()


def set_paused(paused: bool):
    if paused:
        pause_event.set()
    else:
        pause_event.clear()


def is_action_running() -> bool:
    return bool(_action_thread and _action_thread.is_alive())


def connect(port: str, baudrate: int = 1_000_000):
    """连接串口（供 GUI 配置页调用）。"""
    global bus
    with bus_lock:
        if bus is not None:
            return
        bus = ServoBus(port=port, baudrate=baudrate)

    # Start background over-current monitor (best-effort, daemon).
    _start_overcurrent_monitor()


def disconnect():
    """断开串口并释放资源（供 GUI 退出/停止调试调用）。"""
    global bus
    # 断开时不要把“暂停状态”带到下一次连接
    pause_event.clear()
    # 退出时尽量不要因为锁竞争卡死 UI：抢不到锁就做 best-effort 断开
    acquired = bus_lock.acquire(timeout=0.5)
    if not acquired:
        local = bus
        bus = None
        _stop_overcurrent_monitor()
        try:
            if local is not None:
                local.close()
        except Exception:
            pass
        return

    try:
        if bus is None:
            return
        local = bus
        bus = None
        _stop_overcurrent_monitor()
        # Reset protection state on disconnect to avoid carrying a latched trip across sessions.
        reset_overcurrent_trips(None)
        try:
            local.close()
        except Exception:
            pass
    finally:
        bus_lock.release()


def _require_bus():
    if bus is None:
        raise RuntimeError("not connected")


def _oc_filter_current_sample(sid: int, ma_abs: float) -> float | None:
    """Filter out implausible current spikes.

    Returns filtered abs(mA) or None to ignore this sample.
    """
    try:
        v = abs(float(ma_abs))
    except Exception:
        return None
    if v <= 0.0:
        return None
    try:
        if v > float(_oc_filter_phys_max_ma):
            return None
    except Exception:
        pass

    prev = _oc_filter_last_ok_ma.get(int(sid))
    if prev is not None:
        try:
            if (float(v) - float(prev)) > float(_oc_filter_jump_delta_ma):
                return None
        except Exception:
            pass
    _oc_filter_last_ok_ma[int(sid)] = float(v)
    return float(v)


def set_write_params(speed: int | None = None):
    """设置动作/手动写入的 speed（time_ms 固定为 0）。"""
    global _write_speed
    if speed is not None:
        _write_speed = max(0, min(1000, int(speed)))


def get_write_params():
    return _write_speed


def _suggest_wait_seconds(base_delay: float) -> float:
    """根据当前 speed 建议等待时间，避免动作未到位就进入下一步。"""
    speed = get_write_params()
    # 粗略按 speed 估算：speed 越小等待越久（上限避免过度放大）
    if speed and speed > 0 and speed < 1000:
        factor = min(5.0, 1000.0 / float(speed))
        return float(base_delay) * factor + 0.05
    return float(base_delay)

# ======================
# 通信模式（给 GUI 用）
# ======================
COMM_MODE_IDLE = 0
COMM_MODE_ACTION = 1
COMM_MODE_MANUAL = 2
comm_mode = COMM_MODE_IDLE

# 供后台监控线程提示 GUI 状态（由 run_action 注入）
_action_status_cb = None
_action_skip_auto_zero: bool = False
_action_oc_pause_notified: bool = False

# ======================
# 常量
# ======================
ZERO = 2047
MAX = 2900
MIN = 1100

# ======================
# 基础工具
# ======================
def check_abort():
    if stop_event.is_set():
        raise RuntimeError("abort")


def wait_if_paused(step=0.05):
    while pause_event.is_set():
        check_abort()
        time.sleep(step)

def safe_sleep(sec, step=0.02):
    start = time.time()
    while time.time() - start < sec:
        check_abort()
        # 暂停只影响“动作线程”，避免手动滑块也被暂停
        if comm_mode == COMM_MODE_ACTION:
            wait_if_paused(step=max(step, 0.02))
        time.sleep(step)

def move_simultaneous(moves):
    """一组舵机动作（串口互斥 + 微节拍）"""
    check_abort()
    # 暂停只影响“动作线程”，避免手动滑块也被暂停
    if comm_mode == COMM_MODE_ACTION:
        wait_if_paused()
    with bus_lock:
        _require_bus()
        b = bus
        assert b is not None
        speed = get_write_params()

        # ---- Over-current protection (per-servo) ----
        if (not _oc_is_suspended()) and (not _oc_in_grace_period()) and _oc_enabled and _oc_any_limit_configured() and hasattr(b, "sync_read_current_ma"):
            try:
                ids_in_moves = sorted({int(sid) for sid, _pos in moves if 0 <= int(sid) <= 253})
                if ids_in_moves:
                    # Throttle current polling to avoid bus overload during slider drag.
                    global _oc_last_poll_ts
                    now_ts = time.time()
                    do_poll = (now_ts - float(_oc_last_poll_ts)) >= float(_oc_poll_min_interval_sec)
                    currents = {}
                    if do_poll:
                        _oc_last_poll_ts = float(now_ts)
                        currents = b.sync_read_current_ma(ids_in_moves, max_wait=0.06)

                    newly_tripped: list[int] = []
                    for sid in ids_in_moves:
                        ma = currents.get(int(sid)) if isinstance(currents, dict) else None
                        if ma is None:
                            continue
                        ma_f = _oc_filter_current_sample(int(sid), abs(float(ma)))
                        if ma_f is None:
                            continue
                        _oc_last_ma[int(sid)] = float(ma_f)

                        limit = _oc_effective_limit_ma(int(sid))
                        if limit is None:
                            continue
                        reset_threshold = float(limit) * float(_oc_reset_ratio)

                        if float(ma_f) >= float(limit):
                            _oc_over_limit_hits[int(sid)] = int(_oc_over_limit_hits.get(int(sid), 0)) + 1
                            if int(_oc_over_limit_hits[int(sid)]) >= int(_oc_trip_confirm_hits):
                                if int(sid) not in _oc_tripped:
                                    _oc_tripped.add(int(sid))
                                    newly_tripped.append(int(sid))
                                if int(sid) not in _oc_trip_log_once:
                                    _oc_trip_log_once.add(int(sid))
                                    logger.warning(
                                        "Over-current trip: sid=%s current=%.1fmA >= limit=%.1fmA (hits=%s latch=%s)",
                                        int(sid), float(ma_f), float(limit), int(_oc_over_limit_hits.get(int(sid), 0)), bool(_oc_latch)
                                    )
                        else:
                            _oc_over_limit_hits[int(sid)] = 0
                            if (not _oc_latch) and int(sid) in _oc_tripped and float(ma_f) <= reset_threshold:
                                _oc_tripped.discard(int(sid))
                                _oc_trip_log_once.discard(int(sid))
                                _oc_over_limit_hits[int(sid)] = 0
                                logger.info(
                                    "Over-current auto-reset: sid=%s current=%.1fmA <= reset=%.1fmA",
                                    int(sid), float(ma_f), float(reset_threshold)
                                )

                                # Best-effort: re-enable torque when auto-reset.
                                try:
                                    if hasattr(b, "sync_write"):
                                        addr = int(getattr(b, "TORQUE_ENABLE_ADDR", 0x28))
                                        b.sync_write([(int(sid), bytes([0x01]))], start_addr=addr, data_len=1)
                                except Exception:
                                    pass

                    # Best-effort stop for newly tripped servos: disable torque first, then set goal position to current position.
                    if newly_tripped:
                        try:
                            if hasattr(b, "sync_write"):
                                addr = int(getattr(b, "TORQUE_ENABLE_ADDR", 0x28))
                                b.sync_write([(int(sid), bytes([0x00])) for sid in newly_tripped], start_addr=addr, data_len=1)

                            pos_raw = b.sync_read_u16(newly_tripped, start_addr=0x38, max_wait=0.06)
                            for sid in newly_tripped:
                                cur_pos = pos_raw.get(int(sid))
                                if cur_pos is None:
                                    continue
                                b.write_position(int(sid), int(cur_pos), time_ms=0, speed=1000)
                                time.sleep(0.001)
                        except Exception:
                            pass

                    # Filter out tripped servos from the command.
                    if _oc_tripped:
                        moves = [(sid, pos) for (sid, pos) in moves if int(sid) not in _oc_tripped]
            except Exception:
                # Protection should never break control path.
                pass

        # 优先：SYNCWRITE(0x83) 一条指令同时写多个舵机（实时性更好）
        try:
            if hasattr(b, "sync_write_positions") and len(moves) >= 2:
                b.sync_write_positions(moves, time_ms=0, speed=speed)
                return
        except Exception:
            # 失败就回退到逐个写（兼容不同固件/异常情况）
            pass

        # 回退：逐个 WRITEDATA(0x03)
        for sid, pos in moves:
            b.write_position(sid, pos, time_ms=0, speed=speed)
            time.sleep(0.001)  # ★关键：给总线喘气时间


def _start_overcurrent_monitor() -> None:
    global _oc_monitor_thread
    try:
        if _oc_monitor_thread is not None and _oc_monitor_thread.is_alive():
            return
        _oc_monitor_stop_event.clear()

        def _loop():
            global _oc_last_poll_ts
            while not _oc_monitor_stop_event.is_set():
                try:
                    if _oc_is_suspended() or _oc_in_grace_period():
                        time.sleep(0.05)
                        continue

                    if bus is None or (not _oc_enabled) or (not _oc_any_limit_configured()) or (not hasattr(bus, "sync_read_current_ma")):
                        time.sleep(0.10)
                        continue

                    got = bus_lock.acquire(timeout=0.02)
                    if not got:
                        time.sleep(float(_oc_monitor_interval_sec))
                        continue

                    try:
                        # Re-check after acquiring lock: suspension may be enabled while we were waiting.
                        if _oc_is_suspended() or _oc_in_grace_period():
                            time.sleep(0.01)
                            continue

                        b = bus
                        if b is None:
                            time.sleep(0.05)
                            continue

                        # Poll all managed servos for current.
                        currents = b.sync_read_current_ma(_oc_monitor_ids, max_wait=0.08)
                        if not isinstance(currents, dict):
                            time.sleep(float(_oc_monitor_interval_sec))
                            continue

                        # If suspension starts after polling, do not trip based on this sample.
                        if _oc_is_suspended() or _oc_in_grace_period():
                            time.sleep(0.01)
                            continue

                        newly_tripped: list[int] = []

                        for sid in _oc_monitor_ids:
                            ma = currents.get(int(sid))
                            if ma is None:
                                continue
                            ma_f = _oc_filter_current_sample(int(sid), abs(float(ma)))
                            if ma_f is None:
                                continue
                            _oc_last_ma[int(sid)] = float(ma_f)

                            limit = _oc_effective_limit_ma(int(sid))
                            if limit is None:
                                continue
                            reset_threshold = float(limit) * float(_oc_reset_ratio)

                            if ma_f >= float(limit):
                                _oc_over_limit_hits[int(sid)] = int(_oc_over_limit_hits.get(int(sid), 0)) + 1
                                if int(_oc_over_limit_hits[int(sid)]) >= int(_oc_trip_confirm_hits):
                                    if int(sid) not in _oc_tripped:
                                        _oc_tripped.add(int(sid))
                                        newly_tripped.append(int(sid))
                                    if int(sid) not in _oc_trip_log_once:
                                        _oc_trip_log_once.add(int(sid))
                                        logger.warning(
                                            "Over-current trip (monitor): sid=%s current=%.1fmA >= limit=%.1fmA (hits=%s latch=%s)",
                                            int(sid), float(ma_f), float(limit), int(_oc_over_limit_hits.get(int(sid), 0)), bool(_oc_latch)
                                        )
                            else:
                                _oc_over_limit_hits[int(sid)] = 0
                                if (not _oc_latch) and int(sid) in _oc_tripped and ma_f <= reset_threshold:
                                    _oc_tripped.discard(int(sid))
                                    _oc_trip_log_once.discard(int(sid))
                                    try:
                                        if hasattr(b, "sync_write"):
                                            addr = int(getattr(b, "TORQUE_ENABLE_ADDR", 0x28))
                                            b.sync_write([(int(sid), bytes([0x01]))], start_addr=addr, data_len=1)
                                    except Exception:
                                        pass

                        if newly_tripped:
                            # Immediate best-effort stop.
                            try:
                                if hasattr(b, "sync_write"):
                                    addr = int(getattr(b, "TORQUE_ENABLE_ADDR", 0x28))
                                    b.sync_write([(int(sid), bytes([0x00])) for sid in newly_tripped], start_addr=addr, data_len=1)
                            except Exception:
                                pass

                            # Extra: set goal pos to current pos if readable.
                            try:
                                pos_raw = b.sync_read_u16(newly_tripped, start_addr=0x38, max_wait=0.08)
                                for sid in newly_tripped:
                                    cur_pos = pos_raw.get(int(sid))
                                    if cur_pos is None:
                                        continue
                                    b.write_position(int(sid), int(cur_pos), time_ms=0, speed=1000)
                                    time.sleep(0.001)
                            except Exception:
                                pass

                            # If we are in an action, auto-pause the action sequence and skip auto-zero.
                            try:
                                global _action_skip_auto_zero, _action_oc_pause_notified
                                if comm_mode == COMM_MODE_ACTION:
                                    _action_skip_auto_zero = True
                                    pause_event.set()
                                    if (not _action_oc_pause_notified) and callable(_action_status_cb):
                                        _action_oc_pause_notified = True
                                        try:
                                            _action_status_cb("🟠 过流触发：已自动暂停（不回零）")
                                        except Exception:
                                            pass
                            except Exception:
                                pass
                    finally:
                        try:
                            bus_lock.release()
                        except Exception:
                            pass

                    time.sleep(float(_oc_monitor_interval_sec))
                except Exception:
                    # Never let monitor crash the process.
                    time.sleep(0.10)

        _oc_monitor_thread = threading.Thread(target=_loop, daemon=True)
        _oc_monitor_thread.start()
    except Exception:
        pass


def _stop_overcurrent_monitor() -> None:
    global _oc_monitor_thread
    try:
        _oc_monitor_stop_event.set()
        _oc_monitor_thread = None
    except Exception:
        pass

def action_all_zero():
    # 回零动作：应忽略舵机限电流（即使启用过流保护也不要熔断）。
    # 同时若之前因过流禁用了 torque，这里也 best-effort 重新使能，确保能回到零位。
    with suspend_overcurrent_protection(grace_sec=0.8):
        try:
            _best_effort_enable_torque(list(range(1, 18)), lock_timeout=0.20, retries=2)
        except Exception:
            pass
        with bus_lock:
            if bus is None:
                return
            for sid in range(1, 18):
                # 回零默认用“最快”参数，避免用户把速度/时间调慢导致急停不够快
                bus.write_position(sid, ZERO, time_ms=0, speed=500)
                time.sleep(0.001)

def play_sequence(sequence, delay=1.5):
    for step in sequence:
        move_simultaneous(step)
        safe_sleep(_suggest_wait_seconds(delay))

# ======================
# 对外统一执行接口
# ======================
def run_action(action_func, status_cb=None, auto_zero: bool = True):
    global _action_thread, comm_mode

    if _action_thread and _action_thread.is_alive():
        return

    global _action_status_cb, _action_skip_auto_zero, _action_oc_pause_notified
    stop_event.clear()
    pause_event.clear()
    _action_status_cb = status_cb
    _action_skip_auto_zero = (not bool(auto_zero))
    _action_oc_pause_notified = False
    comm_mode = COMM_MODE_ACTION

    def wrapper():
        global comm_mode
        try:
            if status_cb:
                status_cb("🔵 正在执行")

            _require_bus()

            action_func()

            if (not stop_event.is_set()) and (not _action_skip_auto_zero):
                # Auto-zero step should ignore overcurrent protection.
                with suspend_overcurrent_protection(grace_sec=1.2):
                    action_all_zero()
                if status_cb:
                    status_cb("🟢 执行完成")
                safe_sleep(1.5)
            elif (not stop_event.is_set()) and _action_skip_auto_zero:
                # 过流触发的动作：不回零，保持当前状态（通常处于暂停）
                if status_cb:
                    status_cb("🟠 已结束：未回零")

        except RuntimeError:
            pass
        finally:
            comm_mode = COMM_MODE_IDLE
            stop_event.clear()
            pause_event.clear()
            _action_status_cb = None
            if status_cb:
                status_cb("🟢 空闲")

    _action_thread = threading.Thread(target=wrapper, daemon=True)
    _action_thread.start()

# ======================
# 急停
# ======================
def emergency_stop(status_cb=None, *, wait: bool = False):
    global comm_mode
    stop_event.set()
    try:
        _calibration_abort_event.set()
    except Exception:
        pass
    # 急停必须能立即生效：清除暂停，避免动作线程卡在暂停等待
    pause_event.clear()
    comm_mode = COMM_MODE_ACTION

    if status_cb:
        status_cb("🟡 紧急回零")

    def do():
        global comm_mode, _oc_grace_until_ts
        # Emergency zeroing must ignore overcurrent protection completely.
        # Also suspend as early as possible to prevent the monitor thread from tripping while we're starting.
        try:
            with _oc_suspend_lock:
                global _oc_suspend_count
                _oc_suspend_count = int(_oc_suspend_count) + 1
        except Exception:
            pass
        try:
            _oc_grace_until_ts = max(float(_oc_grace_until_ts), float(time.monotonic()) + 2.0)
        except Exception:
            pass
        try:
            # User expectation: after an over-current event, hitting E-Stop should also recover control.
            # So we proactively clear tripped state + best-effort torque re-enable, then zero.
            try:
                reset_overcurrent_trips(None)
            except Exception:
                pass
            # Ensure grace window isn't shortened by reset_overcurrent_trips().
            try:
                _oc_grace_until_ts = max(float(_oc_grace_until_ts), float(time.monotonic()) + 2.0)
            except Exception:
                pass
            action_all_zero()
            time.sleep(2)
        finally:
            try:
                with _oc_suspend_lock:
                    _oc_suspend_count = max(0, int(_oc_suspend_count) - 1)
            except Exception:
                pass
            # Clear abort flags so manual controls work again after E-Stop.
            try:
                stop_event.clear()
            except Exception:
                pass
            try:
                pause_event.clear()
            except Exception:
                pass
            try:
                _calibration_abort_event.clear()
            except Exception:
                pass
            comm_mode = COMM_MODE_IDLE
            if status_cb:
                status_cb("🟢 空闲")

    t = threading.Thread(target=do, daemon=True)
    t.start()
    if bool(wait):
        try:
            t.join(timeout=4.0)
        except Exception:
            pass

# ======================
# 动作定义（完全保留你的）
# ======================
def action_1_finger_traverse():
    sequence = [
        [(1, MAX), (2, MIN)],
        [(1, ZERO), (2, ZERO), (8, MAX), (9, MIN)],
        [(8, ZERO), (9, ZERO), (10, MIN), (11, MAX)],
        [(10, ZERO), (11, ZERO), (6, MIN), (7, MIN)],
        [(6, ZERO), (7, ZERO), (14, MIN), (16, MAX)],
        [(14, ZERO), (16, ZERO)]
    ]
    play_sequence(sequence)

def action_2_fist():
    move_simultaneous([
        (1, MAX), (2, MIN),
        (6, MIN), (7, MIN),
        (8, MAX), (9, MIN),
        (10, MIN), (11, MAX),
        (14, MIN), (16, MAX),
        (13, 2340),
        (15, 2160),
        (3, 2229),
        (12, 2207),
        (5, 1734)
    ])
    safe_sleep(_suggest_wait_seconds(2.0))

def action_3_swing():
    move_simultaneous([(17, 1550)])
    safe_sleep(_suggest_wait_seconds(1.2))
    move_simultaneous([(17, 2450)])
    safe_sleep(_suggest_wait_seconds(1.2))

def action_4_pinch_traverse():
    sequence = [
        [(1, 2697), (2, 1506), (13, 2180), (14, 1442), (15, 1888), (16, 2725)],
        [(1, ZERO), (2, ZERO), (8, 2631), (9, 1404), (13, 1960), (14, 1442), (15, 2023), (16, 2725)],
        [(8, ZERO), (9, ZERO), (10, 1420),(11, 2674), (13, 1892), (14, 1442),(15, 2182), (16, 2725)],
        [(10, ZERO), (11, ZERO),(6, 1217), (7, 1596), (13, 1708),(14, 1415), (15, 2203), (16, 2388)],
    ]
    play_sequence(sequence)

def action_5_left_right():
    move_simultaneous([
        (3, 1867), (4, 1753), (5, 1758),(12, 2316), (15, 2180)
    ])
    safe_sleep(_suggest_wait_seconds(1.2))
    move_simultaneous([
        (3, 2338), (4, 2315), (5, 2248),(12, 1780), (15, 1712)
    ])
    safe_sleep(_suggest_wait_seconds(1.2))

def action_6_zero_only():
    action_all_zero()
