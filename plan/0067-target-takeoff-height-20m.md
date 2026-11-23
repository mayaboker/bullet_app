# Default target-hit takeoff height to 20 m

## Goal

Set the target-hit mission's initial takeoff target to 20 m above the altitude measured at Start.

## Implementation steps

1. Change the target-hit default height constant from 10 m to 20 m.
2. Update the mission guide and root README.
3. Assert the new default in the target mission test and run the App suite.

## Interface changes

The default applied target-hit height reported to Bullet View changes to 20 m. View's manual height override remains available before Start.

## Scope and defaults

Only target-hit's initial height changes. The hover-land profile and LOS navigator target altitude are separate settings.

## Verification

Passed: all 101 Bullet App tests. Bullet View's existing target-height control defaults to 20 m.

## Status

Implemented. Implementation commit: Pending (no commit requested).
