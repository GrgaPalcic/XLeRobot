# DualSense Dual-Arm Teleop

This fork supports one Sony DualSense controller for two calibrated SO100/SO101 follower arms when wheels are not installed. It is intentionally separate from the full XLeRobot teleop scripts because the full `XLerobot` class declares wheel motors and sends base velocity commands.

## Current Machine State

The implementation was grounded on the connected hardware on this PC:

- Controller: `DualSense Wireless Controller` at `/dev/input/js0`.
- Motion sensors: `DualSense Wireless Controller Motion Sensors` at `/dev/input/js1`; ignored for v1.
- Left arm port: `/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0`, currently resolving to `/dev/ttyUSB1`.
- Right arm port: `/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-port0`, currently resolving to `/dev/ttyUSB0`.
- Latest LeRobot motor calibrations:
  - left: `/home/dell/.cache/huggingface/lerobot/calibration/robots/so_follower/xlerobot_left.json`
  - right: `/home/dell/.cache/huggingface/lerobot/calibration/robots/so_follower/xlerobot_right.json`

The latest field run that records those paths is `/home/dell/Documents/xlerobot-so101-stack/field_runs/xlerobot_printed_plate_20260520/run_state.yaml`.

## Setup With uv

Install dependencies:

```bash
uv sync --extra dualsense
```

List detected joystick devices:

```bash
uv run xlerobot-dualsense-probe --list
```

Monitor raw input events:

```bash
uv run xlerobot-dualsense-probe --monitor 15
```

Create a local mapping file:

```bash
uv run xlerobot-dualsense-probe --calibrate
```

The generated mapping is written to `.xlerobot/controller_mappings/dualsense.json`. That directory is ignored by git because mappings can be machine and SDL-version specific.

## Dry Run

Run without connecting to arm motors:

```bash
uv run xlerobot-dualsense-dual-arm --dry-run
```

If no mapping has been generated yet, use the best-effort SDL default:

```bash
uv run xlerobot-dualsense-dual-arm --dry-run --use-default-mapping --duration 10
```

## Robot Run

Connect both calibrated arms, read the current pose, and disconnect without entering the control loop:

```bash
uv run xlerobot-dualsense-dual-arm --connect-only --use-default-mapping
```

Run a short bounded live check:

```bash
uv run xlerobot-dualsense-dual-arm --use-default-mapping --duration 5
```

The default command uses the calibrated robot IDs and stable port paths discovered above:

```bash
uv run xlerobot-dualsense-dual-arm
```

Equivalent explicit command:

```bash
uv run xlerobot-dualsense-dual-arm \
  --left-id xlerobot_left \
  --right-id xlerobot_right \
  --left-port /dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0 \
  --right-port /dev/serial/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-port0
```

The script fails before connecting to motors if the calibration files for the selected IDs are missing.
By default it calls LeRobot with `calibrate=False`, so it uses the existing calibration files and does not start an interactive calibration flow.
Only pass `--write-calibration-if-needed` if LeRobot reports that calibration must be written back to the motors.
On connect, the controller captures the observed joint positions as the software home pose and prints those values.
The script clamps each command to 3 degrees per joint before sending it; change that with `--max-command-step`, or pass `--max-command-step 0` to disable it.
The actual run has no time limit; stop with `Ctrl-C`, which disconnects both arms.

## Controls

- Left stick: left shoulder pan and shoulder lift.
- Right stick: right shoulder pan and shoulder lift.
- L1 + left stick: left elbow flex and wrist flex.
- R1 + right stick: right elbow flex and wrist flex.
- L1 + R1 + left or right stick left/right: wrist roll for that arm.
- L2: close left gripper; L1 + L2 opens it.
- R2: close right gripper; R1 + R2 opens it.
- Mapped reset button: return both software targets to the captured home pose.

Neutral sticks and released triggers hold the current observed joint positions, so releasing the controller stops issuing further movement.

The first implementation tried the upstream Xbox-style Cartesian x/y control. On this machine the already-calibrated arms start near the edge of that simple 2D IK model, so the result was small wiggles and confusing persistent target chasing. The default control surface is now direct joint jog because it is easier to understand and easier to stop.

No D-pad/base input is mapped, and the script never emits `x.vel`, `y.vel`, or `theta.vel`.

## Why IMU Is Not Used

The upstream Xbox controller path uses `pygame` joystick axes, buttons, and hats. It does not use IMU data. On this PC, the DualSense IMU appears as a separate Linux input device (`/dev/input/js1`), not as the primary joystick (`/dev/input/js0`). One gamepad IMU also does not naturally provide independent pose intent for two arms. For v1, the controller behaves like the Xbox path: sticks, buttons, and triggers only.
