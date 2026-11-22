"""Compatibility imports for the shared mission state."""
from .missions.base import Action, Command, MissionBase, Mode, Phase, PHASE_IDS
from .missions.hover_land import DEFAULT_PROFILE, HoverLandMission, HoverLandSettings


class FlightMission(HoverLandMission):
    """Legacy import path for the configured hover/land mission."""

    def __init__(self, mode: Mode | str = Mode.MANUAL, blackboard=None):
        super().__init__(HoverLandSettings.from_yaml(DEFAULT_PROFILE), mode, blackboard)
