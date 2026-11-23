# Slower yaw settle before LOS

## Goal

Keep the drone from entering LOS tracking while its search turn is still moving the truck across the camera image.

## Implementation steps

1. Require a measured yaw speed at or below 1°/s for 1 second while the target remains within the existing 10 px center band.
2. Update the target mission guide and add a focused regression test for the handoff and timer reset.

## Interface changes

None. The settle thresholds are mission constants.

## Verification

`uv run pytest -q tests/test_target_mission.py tests/test_mission_state_machine.py`: 34 passed. `git diff --check` passed.

## Scope and defaults

Change only the settle gate before LOS. Keep search, centering, and LOS control unchanged.

## Status

Implemented. Implementation commit: Pending (no commit requested).
