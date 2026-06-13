"""Dual-arm SO100/SO101 teleoperation with one DualSense controller."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .dualsense import DEFAULT_MAPPING_PATH, DualSenseJoystick, load_mapping


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
DEFAULT_LEFT_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0"
DEFAULT_RIGHT_PORT = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-port0"
DEFAULT_LEFT_ID = "xlerobot_left"
DEFAULT_RIGHT_ID = "xlerobot_right"
DEFAULT_CALIBRATION_DIR = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower"


def inverse_kinematics(x: float, y: float, l1: float = 0.1159, l2: float = 0.1350) -> tuple[float, float]:
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset

    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2
    if r > r_max:
        scale_factor = r_max / r
        x *= scale_factor
        y *= scale_factor
        r = r_max

    r_min = abs(l1 - l2)
    if 0 < r < r_min:
        scale_factor = r_min / r
        x *= scale_factor
        y *= scale_factor
        r = r_min

    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = math.pi - math.acos(cos_theta2)
    beta = math.atan2(y, x)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma

    joint2 = max(-0.1, min(3.45, theta1 + theta1_offset))
    joint3 = max(-0.2, min(math.pi, theta2 + theta2_offset))
    return 90 - math.degrees(joint2), math.degrees(joint3) - 90


def forward_kinematics(joint2_deg: float, joint3_deg: float, l1: float = 0.1159, l2: float = 0.1350) -> tuple[float, float]:
    joint2_rad = math.radians(90 - joint2_deg)
    joint3_rad = math.radians(joint3_deg + 90)
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    theta1 = joint2_rad - theta1_offset
    theta2 = joint3_rad - theta2_offset
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2 - math.pi)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2 - math.pi)
    return x, y


@dataclass
class ArmController:
    name: str
    xy_step: float = 0.004
    degree_step: float = 3.0
    gripper_step: float = 5.0
    gripper_open: float = 90.0
    gripper_closed: float = 2.0
    current_x: float = 0.1629
    current_y: float = 0.1131
    pitch: float = 0.0
    targets: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 90.0,
        }
    )
    home_targets: dict[str, float] | None = None
    home_x: float | None = None
    home_y: float | None = None
    home_pitch: float | None = None

    def capture_home(self, obs: dict[str, Any]) -> None:
        self.home_targets = {}
        for joint in JOINTS:
            key = f"{joint}.pos"
            if key in obs:
                value = float(obs[key])
                self.targets[joint] = value
                self.home_targets[joint] = value
        if {"shoulder_lift", "elbow_flex"}.issubset(self.home_targets):
            self.current_x, self.current_y = forward_kinematics(
                self.targets["shoulder_lift"], self.targets["elbow_flex"]
            )
            self.home_x = self.current_x
            self.home_y = self.current_y
        if {"shoulder_lift", "elbow_flex", "wrist_flex"}.issubset(self.home_targets):
            self.pitch = (
                self.targets["wrist_flex"]
                + self.targets["shoulder_lift"]
                + self.targets["elbow_flex"]
            )
            self.home_pitch = self.pitch

    def sync_targets_to_observation(self, obs: dict[str, Any]) -> None:
        for joint in JOINTS:
            key = f"{joint}.pos"
            if key in obs:
                self.targets[joint] = float(obs[key])
        if {"shoulder_lift.pos", "elbow_flex.pos"}.issubset(obs):
            self.current_x, self.current_y = forward_kinematics(
                float(obs["shoulder_lift.pos"]), float(obs["elbow_flex.pos"])
            )
        if {"shoulder_lift.pos", "elbow_flex.pos", "wrist_flex.pos"}.issubset(obs):
            self.pitch = (
                float(obs["wrist_flex.pos"])
                + float(obs["shoulder_lift.pos"])
                + float(obs["elbow_flex.pos"])
            )

    def reset(self) -> None:
        self.current_x = self.home_x if self.home_x is not None else 0.1629
        self.current_y = self.home_y if self.home_y is not None else 0.1131
        self.pitch = self.home_pitch if self.home_pitch is not None else 0.0
        if self.home_targets is not None:
            self.targets.update(self.home_targets)
        else:
            self.targets.update(
                shoulder_pan=0.0,
                shoulder_lift=0.0,
                elbow_flex=0.0,
                wrist_flex=0.0,
                wrist_roll=0.0,
                gripper=self.gripper_open,
            )

    def apply_input(self, stick_x: float, stick_y: float, modifier: bool, trigger: float) -> None:
        if modifier:
            self.pitch += -stick_y * self.degree_step
            self.targets["wrist_roll"] += stick_x * self.degree_step
        else:
            moved = abs(stick_x) > 0.0 or abs(stick_y) > 0.0
            self.current_x += -stick_y * self.xy_step
            self.current_y += stick_x * self.xy_step
            if moved:
                joint2, joint3 = inverse_kinematics(self.current_x, self.current_y)
                self.targets["shoulder_lift"] = joint2
                self.targets["elbow_flex"] = joint3

        self.targets["wrist_flex"] = -self.targets["shoulder_lift"] - self.targets["elbow_flex"] + self.pitch
        self.targets["gripper"] = self.gripper_open - trigger * (self.gripper_open - self.gripper_closed)

    def apply_live_input(
        self,
        obs: dict[str, Any],
        stick_x: float,
        stick_y: float,
        modifier: bool,
        roll_modifier: bool,
        trigger: float,
    ) -> None:
        self.sync_targets_to_observation(obs)

        if modifier and roll_modifier:
            if abs(stick_x) > 0.0:
                self.targets["wrist_roll"] += stick_x * self.degree_step
        elif modifier:
            if abs(stick_x) > 0.0 or abs(stick_y) > 0.0:
                self.targets["elbow_flex"] += -stick_y * self.degree_step
                self.targets["wrist_flex"] += stick_x * self.degree_step
        elif abs(stick_x) > 0.0 or abs(stick_y) > 0.0:
            self.targets["shoulder_pan"] += stick_x * self.degree_step
            self.targets["shoulder_lift"] += -stick_y * self.degree_step

        if trigger > 0.0:
            direction = 1.0 if modifier else -1.0
            gripper_min = min(self.gripper_open, self.gripper_closed)
            gripper_max = max(self.gripper_open, self.gripper_closed)
            self.targets["gripper"] += direction * trigger * self.gripper_step
            self.targets["gripper"] = max(gripper_min, min(gripper_max, self.targets["gripper"]))

    def action(self, obs: dict[str, Any], kp: float, max_command_step: float | None) -> dict[str, float]:
        action = {}
        for joint in JOINTS:
            key = f"{joint}.pos"
            if key not in obs:
                continue
            current = float(obs[key])
            target = self.targets[joint]
            command = current + kp * (target - current)
            if max_command_step is not None:
                command = max(current - max_command_step, min(current + max_command_step, command))
            action[key] = command
        return action

    def home_summary(self) -> str:
        joints = ", ".join(f"{joint}={self.targets[joint]:.2f}" for joint in JOINTS)
        return f"{self.name} home: {joints}; xy=({self.current_x:.4f},{self.current_y:.4f})"


def import_so_follower():
    try:
        from lerobot.robots.so_follower.so_follower import SOFollower as RobotClass
        from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig as ConfigClass

        return RobotClass, ConfigClass
    except ImportError:
        from lerobot.robots.so_follower.so_follower import SO100Follower as RobotClass
        from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig as ConfigClass

        return RobotClass, ConfigClass


def calibration_file(calibration_dir: Path, robot_id: str) -> Path:
    return calibration_dir / f"{robot_id}.json"


def require_calibration(calibration_dir: Path, robot_id: str, label: str) -> Path:
    path = calibration_file(calibration_dir, robot_id)
    if not path.is_file():
        raise SystemExit(
            f"Missing {label} arm calibration file: {path}\n"
            "The calibrated ids on this machine are expected to be "
            "`xlerobot_left` and `xlerobot_right` under LeRobot's cache."
        )
    return path


def make_robot(port: str, robot_id: str, calibration_dir: Path):
    RobotClass, ConfigClass = import_so_follower()
    return RobotClass(
        ConfigClass(
            port=port,
            id=robot_id,
            calibration_dir=calibration_dir,
        )
    )


def describe_controls(mapping: dict[str, Any]) -> str:
    axes = mapping.get("axes", {})
    buttons = mapping.get("buttons", {})

    def axis(name: str) -> str:
        spec = axes.get(name, {})
        return str(spec.get("index", "?"))

    def button(name: str) -> str:
        return str(buttons.get(name, "?"))

    return "\n".join(
        [
            "Controls:",
            f"  left shoulder pan/lift: left stick axes {axis('left_x')}/{axis('left_y')}",
            f"  right shoulder pan/lift: right stick axes {axis('right_x')}/{axis('right_y')}",
            f"  left elbow/wrist flex: hold L1 button {button('l1')} + left stick up/down/left/right",
            f"  right elbow/wrist flex: hold R1 button {button('r1')} + right stick up/down/left/right",
            f"  wrist roll: hold L1+R1 + left or right stick left/right",
            f"  left gripper: L2 axis {axis('l2')} closes, L1+L2 opens, release holds",
            f"  right gripper: R2 axis {axis('r2')} closes, R1+R2 opens, release holds",
            f"  reset to captured home: button {button('reset')}",
        ]
    )


def targets_as_observation(arm: ArmController) -> dict[str, float]:
    return {f"{joint}.pos": arm.targets[joint] for joint in JOINTS}


def run_dry_loop(
    controller: DualSenseJoystick,
    left_arm: ArmController,
    right_arm: ArmController,
    hz: float,
    duration_s: float | None,
) -> None:
    period = 1.0 / hz
    last_print = 0.0
    start = time.monotonic()
    while True:
        if duration_s is not None and time.monotonic() - start >= duration_s:
            print(f"Finished bounded dry-run after {duration_s:.1f}s.")
            return
        snapshot = controller.snapshot()
        if snapshot["reset"]:
            left_arm.reset()
            right_arm.reset()
        else:
            left_arm.apply_live_input(
                targets_as_observation(left_arm),
                snapshot["left_x"],
                snapshot["left_y"],
                snapshot["l1"],
                snapshot["r1"],
                snapshot["l2"],
            )
            right_arm.apply_live_input(
                targets_as_observation(right_arm),
                snapshot["right_x"],
                snapshot["right_y"],
                snapshot["r1"],
                snapshot["l1"],
                snapshot["r2"],
            )

        now = time.monotonic()
        if now - last_print > 0.25:
            print(
                "dry-run "
                f"left_pan={left_arm.targets['shoulder_pan']:.1f} "
                f"left_lift={left_arm.targets['shoulder_lift']:.1f} "
                f"left_elbow={left_arm.targets['elbow_flex']:.1f} "
                f"right_pan={right_arm.targets['shoulder_pan']:.1f} "
                f"right_lift={right_arm.targets['shoulder_lift']:.1f} "
                f"right_elbow={right_arm.targets['elbow_flex']:.1f} "
                f"left_grip={left_arm.targets['gripper']:.1f} "
                f"right_grip={right_arm.targets['gripper']:.1f} "
                f"reset={snapshot['reset']}"
            )
            last_print = now
        time.sleep(period)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping", type=Path, default=DEFAULT_MAPPING_PATH)
    parser.add_argument("--use-default-mapping", action="store_true")
    parser.add_argument("--joystick-index", type=int)
    parser.add_argument("--left-port", default=DEFAULT_LEFT_PORT)
    parser.add_argument("--right-port", default=DEFAULT_RIGHT_PORT)
    parser.add_argument("--left-id", default=DEFAULT_LEFT_ID)
    parser.add_argument("--right-id", default=DEFAULT_RIGHT_ID)
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_CALIBRATION_DIR)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument(
        "--max-command-step",
        type=float,
        default=3.0,
        help="Maximum per-command joint delta in degrees before sending. Use 0 to disable the cap.",
    )
    parser.add_argument("--xy-step", type=float, default=0.004)
    parser.add_argument("--degree-step", type=float, default=3.0)
    parser.add_argument("--gripper-open", type=float, default=90.0)
    parser.add_argument("--gripper-closed", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true", help="Read the controller but do not connect to robot arms.")
    parser.add_argument("--duration", type=float, help="Stop after this many seconds. By default the loop runs forever.")
    parser.add_argument("--connect-only", action="store_true", help="Connect both arms, read observations, and disconnect.")
    parser.add_argument(
        "--write-calibration-if-needed",
        action="store_true",
        help="Allow LeRobot to write calibration to motors if it detects a mismatch. Default skips prompts.",
    )
    args = parser.parse_args()

    mapping = load_mapping(args.mapping, use_default=args.use_default_mapping)
    left_calib = calibration_file(args.calibration_dir, args.left_id)
    right_calib = calibration_file(args.calibration_dir, args.right_id)
    if left_calib.is_file():
        print(f"Found left calibration: {left_calib}")
    if right_calib.is_file():
        print(f"Found right calibration: {right_calib}")

    controller = DualSenseJoystick(mapping, joystick_index=args.joystick_index)
    info = controller.connect()
    print(f"Using controller {info.index}: {info.name}")
    print(describe_controls(mapping))

    left_arm = ArmController(
        "left",
        xy_step=args.xy_step,
        degree_step=args.degree_step,
        gripper_open=args.gripper_open,
        gripper_closed=args.gripper_closed,
    )
    right_arm = ArmController(
        "right",
        xy_step=args.xy_step,
        degree_step=args.degree_step,
        gripper_open=args.gripper_open,
        gripper_closed=args.gripper_closed,
    )

    try:
        if args.dry_run:
            run_dry_loop(controller, left_arm, right_arm, args.control_hz, args.duration)
            return

        left_calib = require_calibration(args.calibration_dir, args.left_id, "left")
        right_calib = require_calibration(args.calibration_dir, args.right_id, "right")
        print(f"Using left calibration: {left_calib}")
        print(f"Using right calibration: {right_calib}")
        max_command_step = args.max_command_step if args.max_command_step > 0 else None
        left_robot = make_robot(args.left_port, args.left_id, args.calibration_dir)
        right_robot = make_robot(args.right_port, args.right_id, args.calibration_dir)
        left_robot.connect(calibrate=args.write_calibration_if_needed)
        right_robot.connect(calibrate=args.write_calibration_if_needed)
        left_arm.capture_home(left_robot.get_observation())
        right_arm.capture_home(right_robot.get_observation())
        print(left_arm.home_summary())
        print(right_arm.home_summary())
        if args.connect_only:
            print("Connected both arms and read current calibrated observations.")
            return
        print("Connected both SO follower arms. Neutral controls hold current joint positions.")

        period = 1.0 / args.control_hz
        start = time.monotonic()
        while True:
            if args.duration is not None and time.monotonic() - start >= args.duration:
                print(f"Finished bounded run after {args.duration:.1f}s.")
                return
            snapshot = controller.snapshot()
            if snapshot["reset"]:
                left_arm.reset()
                right_arm.reset()
                left_obs = left_robot.get_observation()
                right_obs = right_robot.get_observation()
            else:
                left_obs = left_robot.get_observation()
                right_obs = right_robot.get_observation()
                left_arm.apply_live_input(
                    left_obs,
                    snapshot["left_x"],
                    snapshot["left_y"],
                    snapshot["l1"],
                    snapshot["r1"],
                    snapshot["l2"],
                )
                right_arm.apply_live_input(
                    right_obs,
                    snapshot["right_x"],
                    snapshot["right_y"],
                    snapshot["r1"],
                    snapshot["l1"],
                    snapshot["r2"],
                )
            left_robot.send_action(left_arm.action(left_obs, args.kp, max_command_step))
            right_robot.send_action(right_arm.action(right_obs, args.kp, max_command_step))
            time.sleep(period)
    except KeyboardInterrupt:
        print("Interrupted, disconnecting.")
    except (ConnectionError, RuntimeError) as exc:
        print(f"Robot communication error, disconnecting: {exc}")
    finally:
        controller.disconnect()
        for name in ("left_robot", "right_robot"):
            robot = locals().get(name)
            if robot is not None and getattr(robot, "is_connected", False):
                try:
                    robot.disconnect()
                except Exception as exc:  # noqa: BLE001 - best-effort cleanup after hardware faults.
                    print(f"Warning: failed to disconnect {name}: {exc}")


if __name__ == "__main__":
    main()
