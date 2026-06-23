#!/usr/bin/env python3
"""Keyboard end-effector control for a LeRobot SO101 follower arm.

This script connects to an SO101 follower arm, reads the current joint state,
uses IK to convert small Cartesian end-effector steps into joint targets, and
sends those joint targets to the motors.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path
from typing import Iterable

import numpy as np
from readchar import key, readkey

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


DEFAULT_PORT = "/dev/tty.usbmodem5B790316511"
DEFAULT_ID = "scheda_nera"
DEFAULT_URDF = Path("SO101/so101_new_calib.urdf")
SO101_GITHUB_API = "https://api.github.com/repos/TheRobotStudio/SO-ARM100/contents/Simulation/SO101"
CONTROL_FRAME_DESCRIPTIONS = {
    "base": "URDF base/world axes",
    "tool": "current gripper-frame axes",
    "start-tool": "gripper-frame axes from startup or last h reset",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move an SO101 end effector with keyboard arrows.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="SO101 follower serial port.")
    parser.add_argument("--id", default=DEFAULT_ID, help="LeRobot robot id used for calibration files.")
    parser.add_argument("--urdf-path", type=Path, default=DEFAULT_URDF, help="Path to SO101 URDF.")
    parser.add_argument(
        "--no-download-urdf",
        action="store_true",
        help="Do not auto-download the SO101 URDF/assets when --urdf-path is missing.",
    )
    parser.add_argument("--step-m", type=float, default=0.005, help="Cartesian step per keypress, in meters.")
    parser.add_argument(
        "--control-frame",
        choices=tuple(CONTROL_FRAME_DESCRIPTIONS),
        default="base",
        help="Coordinate frame used for keyboard XYZ steps.",
    )
    parser.add_argument(
        "--max-distance-m",
        type=float,
        default=0.20,
        help="Maximum distance from the startup end-effector position. Use 0 to disable.",
    )
    parser.add_argument(
        "--max-relative-joint-deg",
        type=float,
        default=5.0,
        help="LeRobot safety clip for each joint command relative to the current position.",
    )
    parser.add_argument(
        "--orientation-weight",
        type=float,
        default=0.01,
        help="IK orientation weight. Lower values prioritize position.",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.05,
        help="Short delay after each command so the arm can start moving.",
    )
    parser.add_argument(
        "--ramp-steps",
        type=int,
        default=4,
        help="Number of smaller commands used to smooth each keypress. Use 1 to disable ramping.",
    )
    parser.add_argument(
        "--ramp-s",
        type=float,
        default=0.12,
        help="Total time spent ramping each keypress command.",
    )
    parser.add_argument(
        "--connect-without-calibration",
        action="store_true",
        help="Call robot.connect(calibrate=False). Use only if the robot is already calibrated.",
    )
    return parser.parse_args()


def download_github_directory(api_url: str, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)

    with urllib.request.urlopen(api_url) as response:
        entries = json.loads(response.read().decode("utf-8"))

    for entry in entries:
        entry_path = destination / entry["name"]
        if entry["type"] == "dir":
            download_github_directory(entry["url"], entry_path)
        elif entry["type"] == "file":
            download_url = entry.get("download_url")
            if download_url is None:
                continue
            if entry_path.exists():
                continue
            print(f"Downloading {entry_path}")
            urllib.request.urlretrieve(download_url, entry_path)


def ensure_urdf(urdf_path: Path, allow_download: bool) -> Path:
    urdf_path = urdf_path.expanduser()
    if urdf_path.exists():
        return urdf_path

    if not allow_download:
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    if urdf_path != DEFAULT_URDF:
        raise FileNotFoundError(
            f"URDF not found: {urdf_path}. Auto-download only supports the default {DEFAULT_URDF} path."
        )

    print("SO101 URDF/assets not found; downloading them from TheRobotStudio/SO-ARM100...")
    download_github_directory(SO101_GITHUB_API, DEFAULT_URDF.parent)
    if not urdf_path.exists():
        raise FileNotFoundError(f"Download finished, but URDF is still missing: {urdf_path}")
    return urdf_path


def motor_positions_from_observation(observation: dict[str, object], motor_names: Iterable[str]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for name in motor_names:
        key_name = f"{name}.pos"
        if key_name not in observation:
            raise KeyError(f"Robot observation is missing {key_name!r}")
        positions[key_name] = float(observation[key_name])
    return positions


def positions_array(positions: dict[str, float], joint_names: Iterable[str]) -> np.ndarray:
    return np.array([positions[f"{name}.pos"] for name in joint_names], dtype=float)


def initialize_kinematics(urdf_path: Path, motor_names: list[str]) -> RobotKinematics:
    attempts = [
        motor_names,
        [name for name in motor_names if name != "gripper"],
    ]

    last_error: Exception | None = None
    for joint_names in attempts:
        try:
            return RobotKinematics(
                urdf_path=str(urdf_path),
                target_frame_name="gripper_frame_link",
                joint_names=joint_names,
            )
        except Exception as exc:  # Try the no-gripper chain if the URDF does not expose a gripper joint.
            last_error = exc

    raise RuntimeError(
        f"Could not initialize RobotKinematics with {urdf_path}. "
        f"Last error: {type(last_error).__name__}: {last_error}"
    ) from last_error


def clamp_to_start_workspace(target_position: np.ndarray, start_position: np.ndarray, max_distance: float) -> np.ndarray:
    if max_distance <= 0:
        return target_position

    offset = target_position - start_position
    distance = float(np.linalg.norm(offset))
    if distance <= max_distance or distance == 0:
        return target_position
    return start_position + offset * (max_distance / distance)


def delta_in_base_frame(
    delta: np.ndarray,
    control_frame: str,
    current_pose: np.ndarray,
    start_rotation: np.ndarray,
) -> np.ndarray:
    if control_frame == "base":
        return delta
    if control_frame == "tool":
        return current_pose[:3, :3] @ delta
    if control_frame == "start-tool":
        return start_rotation @ delta
    raise ValueError(f"Unsupported control frame: {control_frame}")


def format_vector(vector: np.ndarray) -> str:
    return f"({vector[0]:+.3f}, {vector[1]:+.3f}, {vector[2]:+.3f})"


def print_pose(label: str, pose: np.ndarray) -> None:
    position = pose[:3, 3]
    rotation = pose[:3, :3]
    print(f"{label}: position in URDF base frame x={position[0]:+.3f} y={position[1]:+.3f} z={position[2]:+.3f}")
    print(
        "  gripper axes in base frame: "
        f"+X={format_vector(rotation[:, 0])} "
        f"+Y={format_vector(rotation[:, 1])} "
        f"+Z={format_vector(rotation[:, 2])}"
    )


def send_ramped_action(
    robot: SO101Follower,
    start_action: dict[str, float],
    target_action: dict[str, float],
    steps: int,
    duration_s: float,
) -> dict[str, float]:
    steps = max(1, steps)
    delay_s = max(0.0, duration_s) / steps
    sent_action: dict[str, float] = {}

    for step in range(1, steps + 1):
        ratio = step / steps
        action = {
            name: start_action[name] + (target_action[name] - start_action[name]) * ratio
            for name in target_action
        }
        sent_action = robot.send_action(action)
        if delay_s > 0 and step < steps:
            time.sleep(delay_s)

    return sent_action


def print_controls(step_m: float, control_frame: str) -> None:
    print()
    print("Controls")
    print(f"  Control frame     : {control_frame} ({CONTROL_FRAME_DESCRIPTIONS[control_frame]})")
    print(f"  Arrow up/down     : move end effector +X/-X by {step_m * 1000:.1f} mm in that frame")
    print(f"  Arrow left/right  : move end effector +Y/-Y by {step_m * 1000:.1f} mm in that frame")
    print(f"  w / s             : move end effector +Z/-Z by {step_m * 1000:.1f} mm in that frame")
    print("  p                 : print current and desired end-effector frames")
    print("  h                 : reset hold pose/workspace center to current pose")
    print("  q or Esc          : quit")
    print()


def key_to_delta(ch: str, step_m: float) -> np.ndarray | None:
    if ch == key.UP:
        return np.array([step_m, 0.0, 0.0], dtype=float)
    if ch == key.DOWN:
        return np.array([-step_m, 0.0, 0.0], dtype=float)
    if ch == key.LEFT:
        return np.array([0.0, step_m, 0.0], dtype=float)
    if ch == key.RIGHT:
        return np.array([0.0, -step_m, 0.0], dtype=float)
    if ch in ("w", "W"):
        return np.array([0.0, 0.0, step_m], dtype=float)
    if ch in ("s", "S"):
        return np.array([0.0, 0.0, -step_m], dtype=float)
    return None


def main() -> int:
    args = parse_args()
    urdf_path = ensure_urdf(args.urdf_path, allow_download=not args.no_download_urdf)

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.id,
            max_relative_target=args.max_relative_joint_deg,
            use_degrees=True,
        )
    )

    motor_names = list(robot.bus.motors.keys())
    kinematics = initialize_kinematics(urdf_path, motor_names)

    print(f"Connecting to SO101 on {args.port}")
    robot.connect(calibrate=not args.connect_without_calibration)

    try:
        observation = robot.get_observation()
        current_positions = motor_positions_from_observation(observation, motor_names)
        desired_positions = current_positions.copy()
        q_desired = positions_array(desired_positions, kinematics.joint_names)
        start_pose = kinematics.forward_kinematics(q_desired)
        start_position = start_pose[:3, 3].copy()
        start_rotation = start_pose[:3, :3].copy()

        print("Connected. The arm will move one small step for each keypress.")
        print("Keep one hand near power/USB disconnect while testing.")
        input("Press ENTER when the workspace is clear and you are ready...")
        print_controls(args.step_m, args.control_frame)
        print_pose("Start pose", start_pose)

        while True:
            ch = readkey()
            if ch in ("q", "Q", key.ESC):
                print("Stopping.")
                break

            if ch in ("h", "H"):
                observation = robot.get_observation()
                current_positions = motor_positions_from_observation(observation, motor_names)
                desired_positions = current_positions.copy()
                q_desired = positions_array(desired_positions, kinematics.joint_names)
                start_pose = kinematics.forward_kinematics(q_desired)
                start_position = start_pose[:3, 3].copy()
                start_rotation = start_pose[:3, :3].copy()
                print("Hold pose and workspace center reset to current end-effector pose.")
                print_pose("Reset pose", start_pose)
                continue

            if ch in ("p", "P"):
                observation = robot.get_observation()
                current_positions = motor_positions_from_observation(observation, motor_names)
                q_current = positions_array(current_positions, kinematics.joint_names)
                current_pose = kinematics.forward_kinematics(q_current)
                q_desired = positions_array(desired_positions, kinematics.joint_names)
                desired_pose = kinematics.forward_kinematics(q_desired)
                print_pose("Current measured pose", current_pose)
                print_pose("Desired hold pose", desired_pose)
                continue

            delta = key_to_delta(ch, args.step_m)
            if delta is None:
                continue

            q_desired = positions_array(desired_positions, kinematics.joint_names)
            desired_pose = kinematics.forward_kinematics(q_desired)
            delta_base = delta_in_base_frame(delta, args.control_frame, desired_pose, start_rotation)

            target_pose = desired_pose.copy()
            target_pose[:3, 3] = clamp_to_start_workspace(
                desired_pose[:3, 3] + delta_base,
                start_position,
                args.max_distance_m,
            )

            q_target = kinematics.inverse_kinematics(
                q_desired,
                target_pose,
                position_weight=1.0,
                orientation_weight=args.orientation_weight,
            )

            action = desired_positions.copy()
            for index, joint_name in enumerate(kinematics.joint_names):
                if joint_name == "gripper":
                    continue
                action[f"{joint_name}.pos"] = float(q_target[index])

            sent_action = send_ramped_action(
                robot,
                desired_positions.copy(),
                action,
                args.ramp_steps,
                args.ramp_s,
            )
            for action_key, value in sent_action.items():
                desired_positions[action_key] = float(value)

            time.sleep(args.settle_s)
            observation = robot.get_observation()
            current_positions = motor_positions_from_observation(observation, motor_names)
            q_current = positions_array(current_positions, kinematics.joint_names)
            current_pose = kinematics.forward_kinematics(q_current)
            ee_position = target_pose[:3, 3]
            print(
                "sent "
                f"x={ee_position[0]:+.3f} y={ee_position[1]:+.3f} z={ee_position[2]:+.3f} "
                f"actual={format_vector(current_pose[:3, 3])} "
                f"joints={sent_action}"
            )

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
