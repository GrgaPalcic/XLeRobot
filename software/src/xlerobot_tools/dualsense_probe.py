"""Probe and calibrate a DualSense controller mapping."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from .dualsense import DEFAULT_MAPPING, DEFAULT_MAPPING_PATH, list_joysticks, monitor_events, require_pygame, save_mapping


AXIS_PROMPTS = [
    ("left_x", "Move the left stick left/right until detected."),
    ("left_y", "Move the left stick up/down until detected."),
    ("right_x", "Move the right stick left/right until detected."),
    ("right_y", "Move the right stick up/down until detected."),
    ("l2", "Squeeze and release L2 until detected."),
    ("r2", "Squeeze and release R2 until detected."),
]

BUTTON_PROMPTS = [
    ("l1", "Press L1."),
    ("r1", "Press R1."),
    ("reset", "Press the button you want to use for reset/return-to-start."),
]


def detect_axis(pygame, joystick, duration_s: float = 4.0) -> tuple[int, str]:
    baseline = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    max_delta = [0.0 for _ in baseline]
    max_value = baseline[:]
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        pygame.event.pump()
        for index in range(joystick.get_numaxes()):
            value = joystick.get_axis(index)
            delta = abs(value - baseline[index])
            if delta > max_delta[index]:
                max_delta[index] = delta
                max_value[index] = value
        if max(max_delta, default=0.0) > 0.65:
            break
        time.sleep(0.01)
    if not max_delta or max(max_delta) < 0.2:
        raise RuntimeError("No axis movement was detected.")
    index = max(range(len(max_delta)), key=max_delta.__getitem__)
    mode = "signed" if baseline[index] < -0.4 and max_value[index] > 0.4 else "positive"
    return index, mode


def detect_button(pygame, duration_s: float = 6.0) -> int:
    start = time.monotonic()
    while time.monotonic() - start < duration_s:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                return int(event.button)
        time.sleep(0.01)
    raise RuntimeError("No button press was detected.")


def calibrate(joystick_index: int | None, output: Path) -> None:
    pygame = require_pygame()
    pygame.init()
    pygame.joystick.init()
    infos = list_joysticks(pygame)
    if not infos:
        raise SystemExit("No pygame joystick devices detected.")
    info = infos[0] if joystick_index is None else next((i for i in infos if i.index == joystick_index), None)
    if info is None:
        raise SystemExit(f"Joystick index {joystick_index} was not detected.")

    joystick = pygame.joystick.Joystick(info.index)
    joystick.init()
    mapping = {
        "schema_version": 1,
        "device_name_contains": "DualSense",
        "source": {
            "device_name": info.name,
            "guid": info.guid,
            "generated_by": "xlerobot-dualsense-probe",
        },
        "axes": {},
        "buttons": {},
    }

    try:
        print(f"Calibrating {info.index}: {info.name}")
        print("Release all controls before each prompt.")
        for name, prompt in AXIS_PROMPTS:
            input(f"\n{name}: {prompt} Press ENTER when ready.")
            index, mode = detect_axis(pygame, joystick)
            kind = "trigger" if name in {"l2", "r2"} else "stick"
            mapping["axes"][name] = {"index": index, "invert": False, "kind": kind, "mode": mode}
            print(f"  detected axis {index} mode={mode}")
            time.sleep(0.5)

        for name, prompt in BUTTON_PROMPTS:
            input(f"\n{name}: {prompt} Press ENTER when ready.")
            index = detect_button(pygame)
            mapping["buttons"][name] = index
            print(f"  detected button {index}")
            time.sleep(0.5)
    finally:
        joystick.quit()
        pygame.joystick.quit()
        pygame.quit()

    save_mapping(output, mapping)
    print(f"\nSaved mapping to {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapping-out", type=Path, default=DEFAULT_MAPPING_PATH)
    parser.add_argument("--joystick-index", type=int)
    parser.add_argument("--list", action="store_true", help="List pygame joystick devices and exit.")
    parser.add_argument("--monitor", type=float, metavar="SECONDS", help="Print live joystick events.")
    parser.add_argument("--calibrate", action="store_true", help="Interactively create a DualSense mapping file.")
    parser.add_argument("--print-default", action="store_true", help="Print the fallback SDL-style mapping.")
    args = parser.parse_args()

    if args.print_default:
        import json

        print(json.dumps(DEFAULT_MAPPING, indent=2, sort_keys=True))
        return

    if args.list:
        for info in list_joysticks():
            print(
                f"{info.index}: name={info.name!r} guid={info.guid} "
                f"axes={info.axes} buttons={info.buttons} hats={info.hats}"
            )
        return

    if args.monitor is not None:
        monitor_events(args.joystick_index, args.monitor)
        return

    if args.calibrate:
        calibrate(args.joystick_index, args.mapping_out)
        return

    parser.print_help()


if __name__ == "__main__":
    main()
