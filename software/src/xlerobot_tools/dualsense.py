"""DualSense input helpers backed by pygame's joystick API."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

DEFAULT_MAPPING_PATH = Path(".xlerobot/controller_mappings/dualsense.json")

DEFAULT_MAPPING: dict[str, Any] = {
    "schema_version": 1,
    "device_name_contains": "DualSense",
    "axes": {
        "left_x": {"index": 0, "invert": False, "kind": "stick"},
        "left_y": {"index": 1, "invert": False, "kind": "stick"},
        "l2": {"index": 2, "invert": False, "kind": "trigger", "mode": "signed"},
        "right_x": {"index": 3, "invert": False, "kind": "stick"},
        "right_y": {"index": 4, "invert": False, "kind": "stick"},
        "r2": {"index": 5, "invert": False, "kind": "trigger", "mode": "signed"},
    },
    "buttons": {
        "l1": 9,
        "r1": 10,
        "reset": 6,
    },
}


@dataclass(frozen=True)
class JoystickInfo:
    index: int
    name: str
    guid: str
    axes: int
    buttons: int
    hats: int


def require_pygame():
    try:
        import pygame  # type: ignore
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "pygame is not installed. Run `uv sync --extra dualsense` and retry with `uv run ...`."
        ) from exc
    return pygame


def load_mapping(path: Path | None, use_default: bool = False) -> dict[str, Any]:
    if path and path.is_file():
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    if use_default:
        return json.loads(json.dumps(DEFAULT_MAPPING))
    mapping_path = path or DEFAULT_MAPPING_PATH
    raise SystemExit(
        f"No DualSense mapping found at {mapping_path}. "
        "Run `uv run xlerobot-dualsense-probe --calibrate` first, "
        "or pass `--use-default-mapping` for a best-effort SDL default."
    )


def save_mapping(path: Path, mapping: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(mapping, f, indent=2, sort_keys=True)
        f.write("\n")


def normalize_stick(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return max(-1.0, min(1.0, value))


def normalize_trigger(value: float, mode: str, deadzone: float) -> float:
    if mode == "signed":
        value = (value + 1.0) / 2.0
    value = max(0.0, min(1.0, value))
    if value < deadzone:
        return 0.0
    return value


class DualSenseJoystick:
    def __init__(
        self,
        mapping: dict[str, Any],
        *,
        joystick_index: int | None = None,
        deadzone: float = 0.12,
    ) -> None:
        self.pygame = require_pygame()
        self.mapping = mapping
        self.joystick_index = joystick_index
        self.deadzone = deadzone
        self.joystick = None

    def connect(self):
        self.pygame.init()
        self.pygame.joystick.init()
        infos = list_joysticks(self.pygame)
        if not infos:
            raise SystemExit("No pygame joystick devices detected.")

        selected = self._select_joystick(infos)
        joystick = self.pygame.joystick.Joystick(selected.index)
        joystick.init()
        self.joystick = joystick
        return selected

    def disconnect(self) -> None:
        if self.joystick is not None:
            self.joystick.quit()
        self.pygame.joystick.quit()
        self.pygame.quit()

    def _select_joystick(self, infos: list[JoystickInfo]) -> JoystickInfo:
        if self.joystick_index is not None:
            for info in infos:
                if info.index == self.joystick_index:
                    return info
            raise SystemExit(f"Joystick index {self.joystick_index} was not detected.")

        needle = str(self.mapping.get("device_name_contains", "DualSense")).lower()
        candidates = [
            info
            for info in infos
            if needle in info.name.lower()
            and "motion" not in info.name.lower()
            and info.axes >= 4
        ]
        if not candidates:
            visible = ", ".join(f"{info.index}:{info.name}" for info in infos)
            raise SystemExit(f"No DualSense joystick candidate found. Visible devices: {visible}")
        return candidates[0]

    def snapshot(self) -> dict[str, Any]:
        if self.joystick is None:
            raise RuntimeError("DualSenseJoystick.connect() must be called first.")

        self.pygame.event.pump()
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        return {
            "axes": axes,
            "buttons": buttons,
            "left_x": self.axis("left_x", axes),
            "left_y": self.axis("left_y", axes),
            "right_x": self.axis("right_x", axes),
            "right_y": self.axis("right_y", axes),
            "l2": self.axis("l2", axes),
            "r2": self.axis("r2", axes),
            "l1": self.button("l1", buttons),
            "r1": self.button("r1", buttons),
            "reset": self.button("reset", buttons),
        }

    def axis(self, name: str, axes: list[float]) -> float:
        spec = self.mapping.get("axes", {}).get(name)
        if spec is None:
            return 0.0
        index = int(spec["index"])
        if index >= len(axes):
            return 0.0
        value = float(axes[index])
        if bool(spec.get("invert", False)):
            value = -value
        if spec.get("kind") == "trigger":
            return normalize_trigger(value, str(spec.get("mode", "signed")), self.deadzone)
        return normalize_stick(value, self.deadzone)

    def button(self, name: str, buttons: list[int]) -> bool:
        index = self.mapping.get("buttons", {}).get(name)
        if index is None:
            return False
        index = int(index)
        return index < len(buttons) and bool(buttons[index])


def list_joysticks(pygame_module=None) -> list[JoystickInfo]:
    pygame = pygame_module or require_pygame()
    if pygame_module is None:
        pygame.init()
        pygame.joystick.init()
    infos = []
    for index in range(pygame.joystick.get_count()):
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        infos.append(
            JoystickInfo(
                index=index,
                name=joystick.get_name(),
                guid=joystick.get_guid(),
                axes=joystick.get_numaxes(),
                buttons=joystick.get_numbuttons(),
                hats=joystick.get_numhats(),
            )
        )
        joystick.quit()
    if pygame_module is None:
        pygame.joystick.quit()
        pygame.quit()
    return infos


def monitor_events(joystick_index: int | None, duration_s: float) -> None:
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
    print(f"Monitoring {info.index}: {info.name} for {duration_s:.1f}s")
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration_s:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    print(f"axis index={event.axis} value={event.value:+.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"button index={event.button} down")
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"button index={event.button} up")
                elif event.type == pygame.JOYHATMOTION:
                    print(f"hat index={event.hat} value={event.value}")
            time.sleep(0.01)
    finally:
        joystick.quit()
        pygame.joystick.quit()
        pygame.quit()
