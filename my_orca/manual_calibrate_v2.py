#!/usr/bin/env python
"""手动标定 — 直读 0~4095 原始值，按 Enter 记录限位。

流程：
  1. 舵机先到 2048（中间值）
  2. 每次 -10 缓慢往伸展方向走，看到头了就按 Enter → 记录下限
  3. 回到 2048
  4. 每次 +10 缓慢往屈曲方向走，看到头了就按 Enter → 记录上限
  5. 写入 calibration.yaml（存储 raw 整数 0–4095）
"""

import sys
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np
import yaml
from orca_core import OrcaHand

RAW_TO_RAD = 2.0 * np.pi / 4096  # 驱动舵机时需要转为弧度
STEP = 10
MIDDLE = 2048
PERIOD = 0.08
CALIB_SPEED = 250  # 标定转速 (~183 RPM)，默认60太慢跟不上


def _set_motor_raw(hand, motor_id: int, raw: int):
    """直接按 raw 值驱动单个舵机（提速）。"""
    raw = max(0, min(4095, raw))
    rad = raw * RAW_TO_RAD
    with hand._motor_lock:
        hand._motor_client.write_desired_pos([motor_id], np.array([rad]), speed=CALIB_SPEED)


def _read_motor_raw(hand, motor_id: int) -> int | None:
    """读单个舵机的 raw 值。"""
    pos_dict = hand.get_motor_pos(as_dict=True)
    rad = pos_dict.get(motor_id)
    if rad is None:
        return None
    return int(rad / RAW_TO_RAD)


def _sweep(hand, motor_id: int, start: int, delta: int, label: str) -> int | None:
    """缓慢走，Enter 时读取舵机实际 raw 位置记录限位。"""
    import msvcrt

    current = start
    while True:
        # 先发指令
        _set_motor_raw(hand, motor_id, current)

        # 等待舵机跟上来，期间轮询键盘
        waited = 0.0
        tick = 0.02
        while waited < PERIOD:
            time.sleep(tick)
            waited += tick
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\r', b'\n'):
                    actual = _read_motor_raw(hand, motor_id)
                    print(f"\n  ✓ {label} 记录: raw {actual}")
                    return actual
                if ch in (b'q', b'Q'):
                    print(f"\n  ⏭ 跳过")
                    return None

        # 显示当前位置
        raw = _read_motor_raw(hand, motor_id)
        print(f"    cmd={current}  actual={raw if raw is not None else '?'}  ", end="\r")

        current += delta
        if current < 0:
            current = 0
        if current > 4095:
            current = 4095


def calibrate_joint(hand, joint: str, motor_id: int) -> tuple[int, int] | None:
    """标定单个关节，返回 (lower_raw, upper_raw) 或 None（跳过）。"""
    print(f"\n{'='*50}")
    print(f"关节: {joint}  →  电机 {motor_id}")
    print(f"{'='*50}")

    # --- 下限（伸展） ---
    print(f"\n  先到中间值 {MIDDLE} ...")
    _set_motor_raw(hand, motor_id, MIDDLE)
    time.sleep(0.5)

    print(f"  ▶ 伸展方向（每次 -{STEP}），到达极限后按 Enter ...")
    print(f"    按 Q 跳过此关节\n")

    lower_raw = _sweep(hand, motor_id, MIDDLE, -STEP, "伸展")
    if lower_raw is None:
        return None

    # --- 上限（屈曲） ---
    print(f"\n  回到中间值 {MIDDLE} ...")
    _set_motor_raw(hand, motor_id, MIDDLE)
    time.sleep(0.5)

    print(f"  ▶ 屈曲方向（每次 +{STEP}），到达极限后按 Enter ...")
    print(f"    按 Q 跳过此关节\n")

    upper_raw = _sweep(hand, motor_id, MIDDLE, +STEP, "屈曲")
    if upper_raw is None:
        return None

    print(f"\n  回到中间值 {MIDDLE} ...")
    _set_motor_raw(hand, motor_id, MIDDLE)
    time.sleep(0.3)

    return lower_raw, upper_raw


def main():
    import argparse
    p = argparse.ArgumentParser(description="手动标定 — 直读 raw 0-4095")
    p.add_argument("--config", type=str, default="config/config.yaml")
    p.add_argument("--joint", type=str, default=None, help="只标定指定关节")
    args = p.parse_args()

    config_path = str(_PROJECT_ROOT / args.config) if not Path(args.config).is_absolute() else args.config

    hand = OrcaHand(config_path=config_path)
    ok, msg = hand.connect()
    print(f"[connect] {msg}")
    if not ok:
        return

    hand.enable_torque()
    hand.set_control_mode(hand.config.control_mode)
    hand.set_max_current(80)
    time.sleep(0.2)

    joints = hand.config.joint_ids
    motor_map = hand.config.joint_to_motor_map
    cal_path = hand.config.calibration_path

    # 加载已有数据
    existing = {}
    try:
        with open(cal_path, encoding='utf-8') as f:
            existing = yaml.safe_load(f) or {}
    except FileNotFoundError:
        pass

    motor_limits = existing.get("motor_limits", {})

    if args.joint:
        joints = [args.joint]

    print(f"\n{'='*60}")
    print("手动标定 — 直接 0~4095")
    print(f"{'='*60}")
    print(f"每次变化 {STEP} raw，按 Enter 记录，按 Q 跳过")
    print(f"电流限制: 80mA（安全）\n")

    for joint in joints:
        motor_id = abs(motor_map[joint])
        result = calibrate_joint(hand, joint, motor_id)
        if result is None:
            continue

        lower_raw, upper_raw = result
        # 直接存 raw 整数
        motor_limits[str(motor_id)] = [int(lower_raw), int(upper_raw)]

        deg_min = (lower_raw / 4095.0) * 360.0
        deg_max = (upper_raw / 4095.0) * 360.0
        print(f"\n  → {joint} 限位: raw [{lower_raw}, {upper_raw}] → deg [{deg_min:.0f}°, {deg_max:.0f}°]")

        # 实时写入 calibration.yaml
        data = {
            "motor_limits": {int(k): v for k, v in motor_limits.items()},
            "calibrated": False,
            "wrist_calibrated": "wrist" in joints,
        }
        with open(cal_path, "w", encoding='utf-8') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    # 全部完成
    data["calibrated"] = True
    with open(cal_path, "w", encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    print(f"\n✅ 标定完成 → {cal_path}")
    print("所有关节已回到中位 2048，按 Enter 断开连接...")
    input()
    hand.set_max_current(hand.config.max_current)
    hand.disconnect()


if __name__ == "__main__":
    main()
