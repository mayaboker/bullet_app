"""Process-wide current observations shared by App mission consumers."""
from dataclasses import dataclass, field
from typing import ClassVar

from beta_client.entities import TelemetrySnapshot
from los_tracking.types import NavigationInput, NavigationResult
from pymavlink import mavutil


@dataclass(slots=True)
class VisualData:
    """Latest accepted target observations; bbox keeps its capture timestamp."""

    bbox: dict | None = None
    contact: bool = False


@dataclass(slots=True)
class FeatureData:
    """Current outputs of target estimation and navigation."""

    navigation_input: NavigationInput | None = None
    navigation_result: NavigationResult | None = None


class Blackboard:
    """One App-process instance; the control loop is its sole writer."""

    _instance: ClassVar['Blackboard | None'] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'telemetry'):
            self.reset()

    def reset(self) -> None:
        """Clear observations before a new App run, including test runs."""
        self.telemetry: TelemetrySnapshot = TelemetrySnapshot()
        self.visual: VisualData = VisualData()
        self.features: FeatureData = FeatureData()
        self.view_alive: bool = False


@dataclass(frozen=True, slots=True)
class StepContext:
    """One control tick's monotonic time and the shared current observations."""

    blackboard: Blackboard
    now_s: float

    @property
    def altitude_m(self) -> float | None:
        hud = self.blackboard.telemetry.vfr_hud
        return None if hud is None else hud.alt

    @property
    def altitude_at_s(self) -> float:
        return self.blackboard.telemetry.vfr_hud_at

    @property
    def firmware_armed(self) -> bool:
        heartbeat = self.blackboard.telemetry.heartbeat
        return bool(heartbeat is not None and
                    heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def view_alive(self) -> bool:
        return self.blackboard.view_alive
