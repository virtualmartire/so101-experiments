#!/usr/bin/env python3
"""Keyboard joint-space control for a LeRobot SO101 follower arm.

Select one of the six joints, then nudge it with the keyboard. Body joints move in
degrees; the gripper moves on a 0–100 scale (closed to open).
"""

from __future__ import annotations

import argparse
import sys
import time

from readchar import key, readkey

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


DEFAULT_PORT = "/dev/tty.usbmodem5A460838151"
DEFAULT_ID = "scheda_nera"

# SO101 follower motor order (matches LeRobot SOFollower bus definition).
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move each SO101 joint individually with the keyboard.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="SO101 follower serial port.")
    parser.add_argument("--id", default=DEFAULT_ID, help="LeRobot robot id used for calibration files.")
    parser.add_argument(
        "--step-deg",
        type=float,
        default=2.0,
        help="Step size per keypress for body joints (degrees).",
    )
    parser.add_argument(
        "--step-gripper",
        type=float,
        default=5.0,
        help="Step size per keypress for the gripper (0=closed, 100=open).",
    )
    parser.add_argument(
        "--max-relative-joint-deg",
        type=float,
        default=None,
        help="LeRobot safety clip per joint command (degrees). Default: disabled.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.05,
        help="Short delay after each command so the arm can start moving.",
    )
    parser.add_argument(
        "--connect-without-calibration",
        action="store_true",
        help="Call robot.connect(calibrate=False). Use only if the robot is already calibrated.",
    )
    return parser.parse_args()


def joint_degree_limits(robot: SO101Follower, motor_name: str) -> tuple[float, float]:
    """Approximate joint limits in degrees from the stored calibration."""
    cal = robot.calibration[motor_name]
    max_res = robot.bus.model_resolution_table[robot.bus.motors[motor_name].model] - 1
    mid = (cal.range_min + cal.range_max) / 2
    min_deg = (cal.range_min - mid) * 360 / max_res
    max_deg = (cal.range_max - mid) * 360 / max_res
    return min_deg, max_deg


def positions_from_observation(observation: dict[str, object], motor_names: list[str]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for name in motor_names:
        key_name = f"{name}.pos"
        if key_name not in observation:
            raise KeyError(f"Robot observation is missing {key_name!r}")
        positions[key_name] = float(observation[key_name])
    return positions


def print_controls(step_deg: float, step_gripper: float) -> None:
    print()
    print("Controls")
    print("  1–6               : select joint")
    print("    1 shoulder_pan  2 shoulder_lift  3 elbow_flex")
    print("    4 wrist_flex    5 wrist_roll     6 gripper")
    print("  [ / ]             : previous / next joint")
    print(f"  Up / Down, w / s  : move selected joint (+/- {step_deg}° or ±{step_gripper} gripper)")
    print("  + / -             : same as Up / Down")
    print("  p                 : print current joint positions")
    print("  q or Esc          : quit")
    print()


def print_status(
    selected: int,
    positions: dict[str, float],
    motor_names: list[str],
    robot: SO101Follower | None = None,
) -> None:
    parts = []
    for i, name in enumerate(motor_names):
        value = positions[f"{name}.pos"]
        unit = "" if name == "gripper" else "°"
        marker = ">" if i == selected else " "
        suffix = ""
        if robot is not None and name != "gripper":
            min_deg, max_deg = joint_degree_limits(robot, name)
            margin = min(value - min_deg, max_deg - value)
            if margin < 5.0:
                suffix = " [LIMIT]"
        parts.append(f"{marker}{i + 1}:{name}={value:+.1f}{unit}{suffix}")
    print("  ".join(parts))


def apply_delta(
    present: float,
    joint_name: str,
    direction: int,
    step_deg: float,
    step_gripper: float,
    robot: SO101Follower,
) -> float:
    step = step_gripper if joint_name == "gripper" else step_deg
    goal = float(present + direction * step)
    if joint_name == "gripper":
        return max(0.0, min(100.0, goal))
    min_deg, max_deg = joint_degree_limits(robot, joint_name)
    return max(min_deg, min(max_deg, goal))


def main() -> int:
    args = parse_args()

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.id,
            max_relative_target=args.max_relative_joint_deg,
            use_degrees=True,
        )
    )

    motor_names = list(robot.bus.motors.keys())
    if motor_names != JOINT_NAMES:
        print(f"Warning: expected joints {JOINT_NAMES}, got {motor_names}")

    print(f"Connecting to SO101 on {args.port}")
    robot.connect(calibrate=not args.connect_without_calibration)

    selected = 0

    try:
        observation = robot.get_observation()
        positions = positions_from_observation(observation, motor_names)

        print("Connected. One joint moves per keypress.")
        print("Keep one hand near power/USB disconnect while testing.")
        input("Press ENTER when the workspace is clear and you are ready...")
        print_controls(args.step_deg, args.step_gripper)
        print_status(selected, positions, motor_names, robot)

        while True:
            ch = readkey()
            if ch in ("q", "Q", key.ESC):
                print("Stopping.")
                break

            if ch in ("p", "P"):
                observation = robot.get_observation()
                positions = positions_from_observation(observation, motor_names)
                print_status(selected, positions, motor_names, robot)
                continue

            if ch == "[":
                selected = (selected - 1) % len(motor_names)
                print_status(selected, positions, motor_names, robot)
                continue

            if ch == "]":
                selected = (selected + 1) % len(motor_names)
                print_status(selected, positions, motor_names, robot)
                continue

            if ch in "123456":
                selected = int(ch) - 1
                print_status(selected, positions, motor_names, robot)
                continue

            direction = 0
            if ch in (key.UP, "w", "W", "+"):
                direction = 1
            elif ch in (key.DOWN, "s", "S", "-"):
                direction = -1

            if direction == 0:
                continue

            observation = robot.get_observation()
            positions = positions_from_observation(observation, motor_names)
            joint_name = motor_names[selected]
            key_name = f"{joint_name}.pos"
            present = positions[key_name]
            goal = apply_delta(present, joint_name, direction, args.step_deg, args.step_gripper, robot)

            if abs(goal - present) < 1e-3:
                if joint_name != "gripper":
                    min_deg, max_deg = joint_degree_limits(robot, joint_name)
                    print(
                        f"{joint_name} already at limit ({present:+.1f}°, "
                        f"range [{min_deg:+.1f}, {max_deg:+.1f}]). "
                        "Move the other direction or recalibrate."
                    )
                continue

            # Only command the selected joint so other joints are not disturbed.
            sent_action = robot.send_action({key_name: goal})
            sent_goal = float(sent_action[key_name])
            time.sleep(args.settle_s)

            observation = robot.get_observation()
            actual = float(observation[key_name])
            if abs(actual - present) < 0.5:
                print(
                    f"WARNING: {joint_name} did not move "
                    f"(present={present:+.1f}, sent={sent_goal:+.1f}, actual={actual:+.1f}). "
                    "Check wiring/motor 3 or recalibrate with: "
                    "lerobot-calibrate --robot.type=so101_follower --robot.port=... --robot.id=my_so101"
                )
            else:
                print(f"moved {joint_name}: {present:+.1f} -> {actual:+.1f} (sent {sent_goal:+.1f})")
            positions = positions_from_observation(observation, motor_names)

    finally:
        robot.disconnect()
        print("Disconnected.")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
        raise SystemExit(130)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
