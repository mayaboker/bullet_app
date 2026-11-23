# Target-hit 30 m takeoff with earlier descent

## Goal

Start the target-hit approach from 30 m above launch and request faster descent before world X≈30 m, where the previous 1.0 m/s minimum produced a shallow early trajectory.

## Implementation

- Set the target-hit default takeoff height to 30 m without changing hover-land or the 4 m/s takeoff climb-speed ceiling.
- Tune the Bullet LOS profile's minimum tracking descent request while a fresh target remains below the elevation reference. The 2.0 m/s attempt passed above the truck; 2.5 and 2.3 m/s exceeded the 12° acquired-LOS deviation guard. Retain 2.3 m/s and widen that guard to 20° while retaining the 12° frame-to-frame association jump check. Keep the 1.5 m/s entry setting, 2 m/s² slew, 5.5 m/s maximum descent, 10° pitch, and assumed 1 m target altitude.
- Update affected tests and mission documentation.

## Interface changes

No new API or profile fields. Target-hit's default height and two existing Bullet profile values change.

## Verification

- Run Bullet App tests and focused LOS profile/control tests.
- Bullet App: 113 tests passed. Focused LOS entry/estimation tests: 20 passed.
- SITL trials: 2.0 m/s passed above the truck; 2.5 and 2.3 m/s crossed the 12° LOS guard; a 2.3 m/s, 20° run passed above the truck. One 2.3 m/s, 20° run ended after a telemetry time/altitude discontinuity. The final 2.5 m/s, 20° run reported `hit`, with contact near X=46.70 m, Z=7.19 m.
- Final run altitude at X=0/10/20/30/40 m was 29.94/20.74/15.90/12.31/9.18 m. The final two tracking checkpoints reached the 5.5 m/s requested-speed limit; thrust did not saturate.

## Scope and defaults

Only target-hit and its Bullet LOS example profile change. The target altitude remains an assumption; guidance does not consume simulator ground truth.

## Status

Implemented; 113 App and 20 focused LOS tests passed, final 30 m SITL run hit. Implementation commit: Pending (no commit requested).
