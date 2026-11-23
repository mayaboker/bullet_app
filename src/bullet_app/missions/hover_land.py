"""Configured takeoff, timed hover, and landing mission."""
from dataclasses import dataclass
from math import isfinite
from pathlib import Path

import yaml

from ..blackboard import StepContext
from .base import MAX_HEIGHT_M, MIN_HEIGHT_M, MissionBase, Mode, Phase

DEFAULT_PROFILE = Path(__file__).with_name('hover_land.yaml')


@dataclass(frozen=True, slots=True)
class HoverLandSettings:
    altitude_m: float
    hover_duration_s: float

    @classmethod
    def from_yaml(cls, path: Path):
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict) or set(data) != {'altitude_m', 'hover_duration_s'}:
            raise ValueError('Hover profile requires altitude_m and hover_duration_s')
        altitude, duration = data['altitude_m'], data['hover_duration_s']
        if (type(altitude) not in (int, float) or not isfinite(altitude)
                or not MIN_HEIGHT_M <= altitude <= MAX_HEIGHT_M):
            raise ValueError('Hover altitude must be 0.1–100 m above launch')
        if type(duration) not in (int, float) or not isfinite(duration) or duration <= 0:
            raise ValueError('Hover duration must be positive and finite')
        return cls(float(altitude), float(duration))


class HoverLandMission(MissionBase):
    def __init__(self, settings: HoverLandSettings, mode: Mode | str = Mode.MANUAL, blackboard=None):
        super().__init__(mode, 'hover-land', settings.altitude_m, settings.hover_duration_s, blackboard)

    def request_height(self, height_m: float) -> None:
        """Reject height edits because the hover profile fixes the flight height."""
        raise ValueError('Height is fixed by the mission profile')

    def vision_alive(self, now: float) -> bool:
        """Report ready because hover-land does not use camera detections."""
        return True

    def _on_hover_complete(self, context: StepContext) -> None:
        self._enter(Phase.LAND, context.now_s)
