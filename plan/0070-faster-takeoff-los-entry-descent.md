# Faster takeoff and LOS entry descent

## Goal

Raise the takeoff-only climb-speed ceiling to 4 m/s and begin the first LOS track with a controlled 1.5 m/s descent target, avoiding the initial positive vertical velocity seen with the corrected 10° pitch.

## Implementation

- Use a 4 m/s speed ceiling only during Takeoff; retain current hover and landing limits and taper through the existing altitude error controller.
- Add a nonnegative LOS entry-descent tuning value with zero default. Set the Bullet profile to 1.5 m/s and its downward reference slew to 2 m/s². Apply entry descent only on first tracking acquisition when the target is below the tracking reference; preserve vertical continuity on later reacquisitions.
- Log requested and applied LOS vertical speed in tracking checkpoints.

## Interface changes

- Add `tuning.los_entry_descent_m_s` to the LOS profile, defaulting to zero. No Bullet App CLI or transport changes.

## Verification

- Bullet App: 113 tests passed. Focused LOS entry tests: 4 passed. The broader LOS suite has pre-existing collection errors from `bullet_app.rc` imports and stale profile-path assertions.
- SITL target-hit: mission reported `hit`. Peak takeoff climb was 3.69 m/s versus 1.52 m/s before. In the first five LOS seconds, altitude stayed below 19.98 m versus a 22.21 m peak before; vertical speed became negative within one second. The descent-feasibility warning still appeared near contact.

## Scope and defaults

Keep 20 m takeoff height, 10° approach pitch, `target_altitude_m: 1.0`, and other throttle limits unchanged. A 4 m/s ceiling is a requested speed, not a guarantee of achieved speed under thrust limits.

## Status

Implemented; 113 App tests, 4 focused LOS tests, and SITL target contact passed. Implementation commit: Pending (no commit requested).
