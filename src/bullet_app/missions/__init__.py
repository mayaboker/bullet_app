"""Startup-selected mission implementations."""
from .base import Action, Command, MissionBase, Mode, Phase, PHASE_IDS
from .hover_land import DEFAULT_PROFILE, HoverLandMission, HoverLandSettings
from .target_hit import TargetMission


def create_mission(options, blackboard=None):
    if options.mission == 'hover-land':
        settings = HoverLandSettings.from_yaml(options.mission_config or DEFAULT_PROFILE)
        return HoverLandMission(settings, options.mode, blackboard)
    if options.mission == 'target-hit':
        from ..cli import DEFAULT_TARGET_PROFILE
        return TargetMission(options.mission_config or DEFAULT_TARGET_PROFILE, options.mode, blackboard)
    raise ValueError(f'Unsupported mission: {options.mission}')
