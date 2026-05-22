#!/usr/bin/env python
"""Single-joint debug tool. Set a target angle and watch the motor move.

Usage:
    python debug_joint.py                           # interactive mode
    python debug_joint.py --joint index_mcp --angle 30   # one-shot
    python debug_joint.py --list                     # list all joints
"""

import argparse
import sys
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from orca_core import OrcaHand


def build_parser():
    p = argparse.ArgumentParser(description="Single-joint debug tool for ORCA Hand")
    p.add_argument("--config", type=str, default="config/config.yaml")
    p.add_argument("--joint", type=str, default=None, help="Joint name to test")
    p.add_argument("--angle", type=float, default=None, help="Target angle (degrees)")
    p.add_argument("--list", action="store_true", help="List all joints")
    p.add_argument("--sweep", action="store_true",
                   help="Sweep through ROM: extend→neutral→flex→neutral")
    p.add_argument("--mock", action="store_true")
    return p


def connect(config_path: str, use_mock: bool):
    if use_mock:
        from orca_core.hardware_hand import MockOrcaHand
        hand = MockOrcaHand(config_path=config_path)
    else:
        hand = OrcaHand(config_path=config_path)
    ok, msg = hand.connect()
    print(f"[connect] {msg}")
    if not ok:
        raise ConnectionError(msg)
    return hand


def interactive_mode(hand):
    """Interactive prompt to set joint angles."""
    joints = hand.config.joint_ids
    neutral = hand.config.neutral_position
    motor_map = hand.config.joint_to_motor_map

    print("\n" + "=" * 60)
    print("INTERACTIVE JOINT DEBUGGER")
    print("=" * 60)
    print("Commands:")
    print("  <joint> <angle>   — set joint to angle (0–360°)")
    print("  list              — show all joints with limits and motor ID")
    print("  neutral           — move all joints to neutral")
    print("  disable           — disable torque")
    print("  enable            — enable torque")
    print("  pos               — read and show current joint positions")
    print("  quit              — exit")
    print()

    # Show joint table
    print(f"{'Joint':<16s} {'Motor':>6s}  {'Limit min':>8s}  {'Limit max':>8s}  {'Neutral':>8s}")
    print("-" * 58)
    for j in joints:
        deg_min, deg_max = hand.config.get_joint_deg_limits(j)
        motor = motor_map.get(j, "?")
        neu = neutral.get(j, 0)
        print(f"{j:<16s} {str(motor):>6s}  {deg_min:>8.0f}  {deg_max:>8.0f}  {neu:>8.0f}")

    hand.init_joints(move_to_neutral=True)
    print("\nHand initialized. Ready for commands.\n")

    try:
        while True:
            raw = input("> ").strip()
            if not raw:
                continue
            parts = raw.split()
            cmd = parts[0].lower()

            if cmd == "quit" or cmd == "q":
                break
            elif cmd == "list":
                print(f"\n{'Joint':<16s} {'Motor':>6s}  {'Limit min':>8s}  {'Limit max':>8s}  {'Neutral':>8s}")
                print("-" * 58)
                for j in joints:
                    deg_min, deg_max = hand.config.get_joint_deg_limits(j)
                    motor = motor_map.get(j, "?")
                    neu = neutral.get(j, 0)
                    print(f"{j:<16s} {str(motor):>6s}  {deg_min:>8.0f}  {deg_max:>8.0f}  {neu:>8.0f}")
                print()
            elif cmd == "neutral":
                print("Moving to neutral...")
                hand.set_neutral_position()
            elif cmd == "disable":
                hand.disable_torque()
                print("Torque disabled.")
            elif cmd == "enable":
                hand.enable_torque()
                print("Torque enabled.")
            elif cmd == "pos":
                pos = hand.get_joint_position().as_dict()
                for j in joints:
                    print(f"  {j:<16s} {pos.get(j, 0):>8.1f}°")
            elif cmd in joints and len(parts) >= 2:
                try:
                    angle = float(parts[1])
                except ValueError:
                    print(f"Invalid angle: {parts[1]}")
                    continue
                deg_min, deg_max = hand.config.get_joint_deg_limits(cmd)
                clamped = max(deg_min, min(deg_max, angle))
                if clamped != angle:
                    print(f"  (clamped to limit: {angle:.1f}° → {clamped:.1f}°)")
                print(f"  Setting {cmd} (motor {motor_map[cmd]}) to {clamped:.1f}° ...")
                hand.set_joint_positions({cmd: clamped})
            else:
                print(f"Unknown command or joint: '{cmd}'. Use 'list' to see joints.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        hand.set_neutral_position(num_steps=30)
        hand.disconnect()


def sweep_mode(hand, joint_name):
    """Sweep a joint through its limits."""
    deg_min, deg_max = hand.config.get_joint_deg_limits(joint_name)
    neutral = hand.config.neutral_position.get(joint_name, 180)

    print(f"\nSweeping {joint_name}: limits [{deg_min:.0f}°, {deg_max:.0f}°], neutral={neutral:.0f}°")
    print("Ctrl+C to abort.\n")

    hand.enable_torque()
    time.sleep(0.1)

    steps = [
        ("EXTEND (min)", deg_min),
        ("NEUTRAL", neutral),
        ("FLEX (max)", deg_max),
        ("NEUTRAL", neutral),
    ]

    try:
        for label, angle in steps:
            print(f"  → {label}: {angle:.0f}° ...")
            hand.set_joint_positions({joint_name: angle}, num_steps=30, step_size=0.03)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted.")


def main():
    args = build_parser().parse_args()
    config_path = str(_PROJECT_ROOT / args.config) if not Path(args.config).is_absolute() else args.config

    hand = connect(config_path, args.mock)

    try:
        if args.list:
            print(f"\n{'Joint':<16s} {'Motor':>6s}  {'Limit min':>8s}  {'Limit max':>8s}  {'Neutral':>8s}")
            print("-" * 58)
            for j in hand.config.joint_ids:
                deg_min, deg_max = hand.config.get_joint_deg_limits(j)
                motor = hand.config.joint_to_motor_map.get(j, "?")
                neu = hand.config.neutral_position.get(j, 0)
                print(f"{j:<16s} {str(motor):>6s}  {deg_min:>8.0f}  {deg_max:>8.0f}  {neu:>8.0f}")
            return

        if args.sweep:
            joint = args.joint or input("Joint name: ").strip()
            if joint not in hand.config.joint_ids:
                print(f"Unknown joint: {joint}")
                return
            sweep_mode(hand, joint)
        elif args.joint and args.angle is not None:
            # One-shot
            hand.init_joints()
            print(f"Setting {args.joint} to {args.angle:.1f}° ...")
            hand.set_joint_positions({args.joint: args.angle}, num_steps=20, step_size=0.02)
            time.sleep(0.5)
        else:
            interactive_mode(hand)
    finally:
        hand.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
