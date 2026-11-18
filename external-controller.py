#!/usr/bin/env python3
"""Move an SO101 end effector with commands from a local process.

Commands are UTF-8 lines sent to 127.0.0.1:8765. "left" and "right" move
the arm, while "rest" leaves it at its current target. Each command receives
an "ok" response when processed, or an "error" response when unsupported.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path

import dotenv

dotenv.load_dotenv("./.env")

import numpy as np

import keyboard_ee_control as ee
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

COMMAND_HOST = "127.0.0.1"
DEFAULT_COMMAND_PORT = 8765
SUPPORTED_COMMANDS = ("left", "right", "rest")


def parseArgs() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move an SO101 end effector with commands from a local process.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=ee.PORT, help="SO101 follower serial port.")
    parser.add_argument("--id", default=ee.ROBOT_ID, help="LeRobot robot id used for calibration files.")
    parser.add_argument("--urdf-path", type=Path, default=ee.DEFAULT_URDF, help="Path to SO101 URDF.")
    parser.add_argument("--command-port", type=int, default=DEFAULT_COMMAND_PORT, help="Local TCP command port.")
    parser.add_argument("--step-m", type=float, default=0.005, help="Cartesian step per command, in meters.")
    parser.add_argument(
        "--control-frame", choices=tuple(ee.CONTROL_FRAME_DESCRIPTIONS), default="base",
        help="Coordinate frame used for Cartesian steps.",
    )
    parser.add_argument(
        "--max-distance-m", type=float, default=0.0,
        help="Maximum distance from the startup position. Use 0 to disable.",
    )
    parser.add_argument(
        "--max-relative-joint-deg", type=float, default=5.0,
        help="Safety clip for each joint command relative to the current position.",
    )
    parser.add_argument("--orientation-weight", type=float, default=0.01, help="IK orientation weight.")
    parser.add_argument("--settle-s", type=float, default=0.05, help="Delay after each command.")
    parser.add_argument("--ramp-steps", type=int, default=6, help="Ramped commands per external command.")
    parser.add_argument("--ramp-s", type=float, default=0.2, help="Total ramp time per external command.")
    parser.add_argument(
        "--no-download-urdf", action="store_true",
        help="Do not auto-download URDF/assets when --urdf-path is missing.",
    )
    parser.add_argument(
        "--connect-without-calibration", action="store_true",
        help="Connect without calibration; use only if the robot is already calibrated.",
    )
    return parser.parse_args()


def commandToDelta(command: str, step_m: float) -> np.ndarray:
    if command == "left":
        return np.array([0.0, step_m, 0.0], dtype=float)
    if command == "right":
        return np.array([0.0, -step_m, 0.0], dtype=float)
    raise ValueError(f"Unsupported command: {command!r}")


class EndEffectorController:
    def __init__(self, robot: SO101Follower, kinematics: ee.RobotKinematics, args: argparse.Namespace):
        self.robot = robot
        self.kinematics = kinematics
        self.args = args
        self.motor_names = list(robot.bus.motors.keys())
        observation = robot.get_observation()
        self.desired_positions = ee.motor_positions_from_observation(observation, self.motor_names)
        q_desired = ee.positions_array(self.desired_positions, kinematics.joint_names)
        start_pose = kinematics.forward_kinematics(q_desired)
        self.start_position = start_pose[:3, 3].copy()
        self.start_rotation = start_pose[:3, :3].copy()

    def calculateTargetPose(self, command: str) -> tuple[np.ndarray, np.ndarray]:
        q_desired = ee.positions_array(self.desired_positions, self.kinematics.joint_names)
        desired_pose = self.kinematics.forward_kinematics(q_desired)
        delta = commandToDelta(command, self.args.step_m)
        delta_base = ee.delta_in_base_frame(
            delta, self.args.control_frame, desired_pose, self.start_rotation,
        )
        target_pose = desired_pose.copy()
        target_pose[:3, 3] = ee.clamp_to_start_workspace(
            desired_pose[:3, 3] + delta_base,
            self.start_position,
            self.args.max_distance_m,
        )
        return q_desired, target_pose

    def calculateAction(self, q_desired: np.ndarray, target_pose: np.ndarray) -> dict[str, float]:
        q_target = self.kinematics.inverse_kinematics(
            q_desired,
            target_pose,
            position_weight=1.0,
            orientation_weight=self.args.orientation_weight,
        )
        action = self.desired_positions.copy()
        for index, joint_name in enumerate(self.kinematics.joint_names):
            if joint_name == "gripper":
                continue
            action[f"{joint_name}.pos"] = float(q_target[index])
        return action

    def move(self, command: str) -> None:
        q_desired, target_pose = self.calculateTargetPose(command)
        action = self.calculateAction(q_desired, target_pose)
        sent_action = ee.send_ramped_action(
            self.robot,
            self.desired_positions.copy(),
            action,
            self.args.ramp_steps,
            self.args.ramp_s,
        )
        for action_key, value in sent_action.items():
            self.desired_positions[action_key] = float(value)
        time.sleep(self.args.settle_s)
        self.printMovement(target_pose, sent_action)

    def printMovement(self, target_pose: np.ndarray, sent_action: dict[str, float]) -> None:
        observation = self.robot.get_observation()
        current_positions = ee.motor_positions_from_observation(observation, self.motor_names)
        q_current = ee.positions_array(current_positions, self.kinematics.joint_names)
        current_pose = self.kinematics.forward_kinematics(q_current)
        target_position = target_pose[:3, 3]
        print(
            "sent "
            f"x={target_position[0]:+.3f} y={target_position[1]:+.3f} z={target_position[2]:+.3f} "
            f"actual={ee.format_vector(current_pose[:3, 3])} joints={sent_action}",
            flush=True,
        )


def processCommand(controller: EndEffectorController, command: str) -> str:
    if command not in SUPPORTED_COMMANDS:
        return f"error: expected 'left', 'right', or 'rest', received {command!r}\n"
    if command == "rest":
        return "ok\n"
    controller.move(command)
    return "ok\n"


def sendResponse(connection: socket.socket, response: str) -> bool:
    try:
        connection.sendall(response.encode("utf-8"))
    except (BrokenPipeError, ConnectionResetError):
        return False
    return True


def processConnection(controller: EndEffectorController, connection: socket.socket) -> None:
    with connection.makefile("r", encoding="utf-8") as command_stream:
        for raw_command in command_stream:
            command = raw_command.strip().lower()
            if not command:
                continue
            print(f"Received {command!r}", flush=True)
            response = processCommand(controller, command)
            if not sendResponse(connection, response):
                print("External process disconnected before the response was sent.", flush=True)
                break


def printControls(args: argparse.Namespace) -> None:
    print()
    print("External controls")
    print(f"  Listening on  : {COMMAND_HOST}:{args.command_port} (local processes only)")
    print("  Protocol      : one UTF-8 command per line; one response per command")
    print(f"  Control frame : {args.control_frame}")
    print(f"  left / right  : +Y / -Y by {args.step_m * 1000:.1f} mm in that frame")
    print("  rest          : do not issue a new movement")
    print("  Ctrl+C        : disconnect and quit")
    print()


def serveCommands(controller: EndEffectorController) -> None:
    address = (COMMAND_HOST, controller.args.command_port)
    with socket.create_server(address) as command_server:
        print(f"Waiting for commands on {COMMAND_HOST}:{controller.args.command_port}...", flush=True)
        while True:
            connection, _ = command_server.accept()
            with connection:
                processConnection(controller, connection)


def runController(robot: SO101Follower, kinematics: ee.RobotKinematics, args: argparse.Namespace) -> None:
    controller = EndEffectorController(robot, kinematics, args)
    print("Connected. The arm will move one small step for each external command.")
    print("Keep one hand near power/USB disconnect while testing.")
    input("Press ENTER when the workspace is clear and you are ready...")
    printControls(args)
    serveCommands(controller)


def main() -> int:
    args = parseArgs()
    urdf_path = ee.ensure_urdf(args.urdf_path, allow_download=not args.no_download_urdf)
    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.id,
            max_relative_target=args.max_relative_joint_deg,
            use_degrees=True,
        )
    )
    motor_names = list(robot.bus.motors.keys())
    kinematics = ee.initialize_kinematics(urdf_path, motor_names)
    print(f"Connecting to SO101 on {args.port}")
    robot.connect(calibrate=not args.connect_without_calibration)
    try:
        runController(robot, kinematics, args)
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
