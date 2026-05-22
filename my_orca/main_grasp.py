#!/usr/bin/env python
"""Camera-driven ORCA Hand grasping demo.

Captures hand landmarks from a webcam via MediaPipe, maps them to
ORCA Hand joint angles, and sends commands to the physical hand
over Feetech servos on a Windows COM port.

Usage:
    python main_grasp.py                          # default config + camera 0
    python main_grasp.py --config config/config.yaml
    python main_grasp.py --camera 1 --no-auto-grasp
    python main_grasp.py --mock                   # simulation only (no hardware)

Controls:
    ESC         — quit
    SPACE       — toggle auto-grasp mode
    G           — trigger grasp (close hand to predefined grip)
    N           — move hand to neutral
    T           — toggle torque
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Ensure my_orca is on sys.path so `import orca_core` resolves
_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from orca_core import OrcaHand, OrcaJointPositions

# ---- grasp presets (servo degrees, neutral = 180°) --------------------------
# 范围 = 标定限位内，deg_min = 伸展, deg_max = 屈曲
# 反转关节已自动映射：低度=伸, 高度=屈

GRASP_OPEN = {
    # 五指张开：全部伸展，外展拉开
    "thumb_mcp": 180, "thumb_abd": 213, "thumb_pip": 180, "thumb_dip": 180,
    "index_abd": 180, "index_mcp": 180, "index_pip": 180,
    "middle_abd": 180, "middle_mcp": 180, "middle_pip": 180,
    "ring_abd": 180, "ring_mcp": 180, "ring_pip": 180,
    "pinky_abd": 180, "pinky_mcp": 180, "pinky_pip": 180,
    "wrist": 180,
}

GRASP_POWER = {
    # 握拳：全部屈曲，手指并拢
    "thumb_mcp": 183, "thumb_abd": 183, "thumb_pip": 128, "thumb_dip": 118,
    "index_abd": 186, "index_mcp": 132, "index_pip": 130,
    "middle_abd": 174, "middle_mcp": 130, "middle_pip": 127,
    "ring_abd": 170, "ring_mcp": 143, "ring_pip": 131,
    "pinky_abd": 160, "pinky_mcp": 136, "pinky_pip": 147,
    "wrist": 180,
}

GRASP_PINCH = {
    # 指尖捏取：拇指与食指指尖相对，其余指自然微屈
    "thumb_mcp": 195, "thumb_abd": 155, "thumb_pip": 180, "thumb_dip": 170,
    "index_abd": 190, "index_mcp": 170, "index_pip": 170,
    "middle_abd": 175, "middle_mcp": 135, "middle_pip": 135,
    "ring_abd": 185, "ring_mcp": 142, "ring_pip": 140,
    "pinky_abd": 175, "pinky_mcp": 130, "pinky_pip": 135,
    "wrist": 180,
}

GRASP_PERSIMMON = {
    # 抓取柿子娃娃
    "thumb_mcp": 197, "thumb_abd": 150, "thumb_pip": 188, "thumb_dip": 143,
    "index_abd": 178, "index_mcp": 132, "index_pip": 164,
    "middle_abd": 178, "middle_mcp": 138, "middle_pip": 149,
    "ring_abd": 183, "ring_mcp": 158, "ring_pip": 138,
    "pinky_abd": 201, "pinky_mcp": 142, "pinky_pip": 136,
    "wrist": 215,
}

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Camera-driven ORCA Hand grasping demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--config", type=str, default="config/config.yaml",
        help="Path to config.yaml (default: config/config.yaml)",
    )
    p.add_argument("--camera", type=int, default=0, help="Camera device ID")
    p.add_argument(
        "--mock", action="store_true",
        help="Use MockOrcaHand (simulation, no hardware required)",
    )
    p.add_argument(
        "--no-auto-grasp", action="store_true",
        help="Disable auto-grasp detection; manual key-press only",
    )
    p.add_argument(
        "--grasp-threshold", type=float, default=150.0,
        help="Flexion threshold in servo degrees for auto-grasp detection",
    )
    p.add_argument(
        "--mode", type=str, default="track",
        choices=["track", "power", "pinch", "cycle"],
        help="Demo mode: track=camera teleop, power=power grasp, pinch=pinch grasp, "
             "cycle=cycle through grasp types",
    )
    p.add_argument(
        "--skip-init", action="store_true",
        help="Skip joint initialization (use when hand is already calibrated)",
    )
    return p


def connect_hand(config_path: str, use_mock: bool = False):
    """Create and connect an OrcaHand (or MockOrcaHand)."""
    if use_mock:
        from orca_core.hardware_hand import MockOrcaHand
        hand = MockOrcaHand(config_path=config_path)
    else:
        hand = OrcaHand(config_path=config_path)

    success, msg = hand.connect()
    print(f"[connect] {msg}")
    if not success:
        raise ConnectionError(msg)
    return hand


def _fast_set_preset(hand, preset: dict, name: str):
    """Write a preset grasp using fast sync write (no interpolation).

    Suspends overcurrent protection during the move to prevent false trips
    when moving all 17 servos simultaneously.
    """
    print(f" → {name} (fast sync)")
    # Convert int values to float for OrcaJointPositions
    float_preset = {k: float(v) for k, v in preset.items()}
    joint_pos = OrcaJointPositions.from_dict(float_preset)
    joint_pos = hand.config.clamp_joint_positions(joint_pos)
    with hand.overcurrent.suspend(grace_sec=0.8):
        hand._set_joint_positions(joint_pos)


def run_tracking_loop(hand, tracker, mapper, args):
    """Main teleoperation loop: camera → joints → hand."""
    from hand_mapper import detect_grasp_state

    auto_grasp = not args.no_auto_grasp
    current_mode = "track"
    hand_connected = True

    print("\nCamera teleop started. Press ESC to quit, SPACE=toggle auto-grasp, G=grasp, N=neutral, R=reset OC.\n")

    frame_count = 0
    last_cmd_time = time.time()
    cmd_interval = 0.03  # ~30 Hz command rate
    last_oc_display = 0.0

    try:
        while True:
            tracker.read_frame()
            key = tracker.show("ORCA Hand — Camera Grasp Demo", wait_ms=1)
            if key == 27:  # ESC
                break

            frame_count += 1
            lm = tracker.landmarks
            wl = tracker.world_landmarks

            # ---- info overlay -------------------------------------------
            info_lines = [
                f"Mode: {current_mode} | Auto-grasp: {'ON' if auto_grasp else 'OFF'}",
                f"Hand: {'detected' if lm is not None else 'searching...'}",
                f"Connected: {hand_connected}",
            ]

            # Overcurrent status (throttled)
            now = time.time()
            if now - last_oc_display > 0.5:
                last_oc_display = now
                oc = hand.overcurrent
                if oc.enabled:
                    tripped = oc.tripped_ids
                    if tripped:
                        info_lines.append(
                            f"OC TRIP: {sorted(tripped)} (press R to reset)")
                    else:
                        # Show peak current
                        currents = oc.last_currents
                        if currents:
                            peak_id = max(currents, key=lambda k: currents[k])
                            peak_ma = currents[peak_id]
                            info_lines.append(
                                f"OC: OK | peak ID{peak_id}={peak_ma:.0f}mA")
                        else:
                            info_lines.append("OC: enabled (no data yet)")
                else:
                    info_lines.append("OC: disabled")

            if lm is not None and (time.time() - last_cmd_time) >= cmd_interval:
                joint_angles = mapper.compute_joint_angles(lm, wl)

                is_grasping, strength = detect_grasp_state(
                    joint_angles, args.grasp_threshold
                )

                if auto_grasp and is_grasping:
                    current_mode = "grasp"
                elif auto_grasp and not is_grasping:
                    current_mode = "track"

                # Apply angle multiplier for grasp strength
                if current_mode == "grasp":
                    for k in joint_angles:
                        if "pip" in k or "mcp" in k or "dip" in k:
                            joint_angles[k] = min(joint_angles[k] * 1.15, 360.0)

                try:
                    hand.set_joint_positions(joint_angles, num_steps=1)
                except Exception:
                    hand_connected = False

                info_lines.append(
                    f"Joints(deg): idx_mcp={joint_angles.get('index_mcp', 0):.0f} "
                    f"th_mcp={joint_angles.get('thumb_mcp', 0):.0f}"
                )
                info_lines.append(
                    f"Grasp: {'YES' if is_grasping else 'no'} "
                    f"(strength={strength:.2f})"
                )

                last_cmd_time = time.time()

            tracker.draw_info(info_lines)

            # ---- keyboard shortcuts ----------------------------------------
            if key == ord(" "):
                auto_grasp = not auto_grasp
                print(f"Auto-grasp: {'ON' if auto_grasp else 'OFF'}")
            elif key == ord("g"):
                _fast_set_preset(hand, GRASP_POWER, "Power Grasp")
            elif key == ord("p"):
                _fast_set_preset(hand, GRASP_PINCH, "Pinch Grasp")
            elif key == ord("n"):
                print("Moving to neutral (fast, OC suspended).")
                hand.set_neutral_position()
            elif key == ord("o"):
                _fast_set_preset(hand, GRASP_OPEN, "Open Hand")
            elif key == ord("t"):
                print("Toggling torque...")
                try:
                    hand.disable_torque()
                    time.sleep(0.2)
                    hand.enable_torque()
                except Exception:
                    pass
            elif key == ord("r"):
                print("Resetting overcurrent trips...")
                hand.overcurrent.reset_trips(None)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Shutting down...")



def run_cycle_demo(hand, args):
    """Cycle through grasp presets (no camera needed)."""
    presets = [("OPEN", GRASP_OPEN), ("PINCH", GRASP_PINCH), ("POWER", GRASP_POWER)]
    idx = 0

    print("\nCycle demo: press SPACE to cycle grasps, ESC to quit.\n")

    try:
        while True:
            name, pose = presets[idx]
            print(f" → {name}")
            hand.set_joint_positions(pose, num_steps=20, step_size=0.02)

            canvas = 255 * np.ones((200, 400, 3), dtype=np.uint8)
            cv2.putText(canvas, f"GRASP: {name}", (50, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)
            cv2.putText(canvas, "SPACE=next  ESC=quit", (50, 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
            cv2.imshow("ORCA Hand — Cycle Demo", canvas)

            key = cv2.waitKey(0) & 0xFF
            if key == 27:
                break
            elif key == ord(" "):
                idx = (idx + 1) % len(presets)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


def main():
    args = build_parser().parse_args()

    config_path = str(_PROJECT_ROOT / args.config) if not Path(args.config).is_absolute() else args.config

    print(f"Config: {config_path}")
    print(f"Mock: {args.mock}")

    # 1. Connect
    hand = connect_hand(config_path, use_mock=args.mock)

    try:
        # 2. Initialize joints
        if not args.skip_init:
            print("Initializing joints...")
            hand.init_joints()

        # 3. Run demo mode
        if args.mode == "cycle":
            run_cycle_demo(hand, args)
        elif args.mode in ("power", "pinch","persimmon"):
            if args.mode == "persimmon":
                pose = GRASP_PERSIMMON
            if args.mode == "power":
                pose = GRASP_POWER
            if args.mode == "pinch":
                pose = GRASP_PINCH
            print(f"Executing {args.mode} grasp...")
            hand.set_joint_positions(pose, num_steps=30, step_size=0.02)
            input("Press Enter to open hand and exit...")
            hand.set_joint_positions(GRASP_OPEN, num_steps=30, step_size=0.02)
        else:
            # Camera tracking mode
            from hand_tracker import HandTracker
            from hand_mapper import HandToOrcaMapper

            with HandTracker(camera_id=args.camera) as tracker:
                inverted = {j for j, inv in hand.config.joint_inversion_dict.items() if inv}
                mapper = HandToOrcaMapper(
                    flip_hand=False,
                    limits_dict=hand.config.joint_limits_dict,
                    inverted_joints=inverted,
                    ema=0.75,  # Smooth jitter from camera tracking
                )
                run_tracking_loop(hand, tracker, mapper, args)

    finally:
        print("Disconnecting hand...")
        try:
            hand.set_neutral_position(num_steps=20, step_size=0.02)
        except Exception:
            pass
        hand.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
