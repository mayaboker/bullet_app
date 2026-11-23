# Preserve throttle at LOS handoff

## Goal

Prevent the throttle drop when target-hit switches from altitude hold to LOS guidance.

## Implementation steps

1. Capture the active altitude-controller PWM on SETTLE-to-LOS transition.
2. Use that PWM as the LOS nominal-hover baseline; add the mapped difference between requested and nominal hover thrust.
3. Preserve existing PWM bounds and use the current fixed baseline for directly constructed LOS missions.
4. Document the handoff in the target-hit guide and README.

## Interface changes

No external configuration or wire-format change. The private demand mapper accepts the captured hover PWM.

## Scope and defaults

Only target-hit LOS thrust mapping changes. The baseline is captured once at LOS entry and remains fixed during that LOS attempt.

## Verification

Passed: 13 focused target-mission tests and all 101 App tests.

## Status

Implemented. Implementation commit: Pending (no commit requested).
