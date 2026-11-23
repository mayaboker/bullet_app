# Keep LOS descent moving while the target is below reference

## Goal

Remove the X≈10–20 m altitude plateau in the Bullet SITL approach without using simulator ground truth or assuming the truck's 6 m contact height.

## Implementation

- Add `tuning.los_min_track_descent_m_s` with zero default and validation against `max_descent_m_s`; set the Bullet profile to 1.0 m/s.
- During active TRACK with usable visual bearing below the elevation reference, cap the requested vertical speed at −1.0 m/s before normal speed limits and slew. Permit faster descent. Release the cap when the bearing reaches or rises above the reference, or the navigator enters acquisition, recovery, or commit.
- Preserve the existing 1.5 m/s entry descent, 10° pitch, 4 m/s takeoff ceiling, and assumed `target_altitude_m: 1.0`.

## Interface changes

The LOS profile accepts `tuning.los_min_track_descent_m_s`. No App transport or CLI changes.

## Verification

- Seven focused LOS tests passed, including integral freeze, release, and configuration bounds; 20 LOS entry/estimation tests and all 113 Bullet App tests passed. The broader LOS suite retains pre-existing collection errors from removed `bullet_app.rc` imports.
- SITL reported `hit`. Altitude at X=10/20/30 m changed from 16.52/15.68/13.10 m to 15.60/13.38/11.64 m. The X=10–20 m drop increased from 0.84 to 2.22 m, while the X=30–40 m drop fell from 4.45 to 1.85 m. The descent-feasibility warning appeared about 0.4 s later; contact occurred near X=49.14 m, Z=7.38 m.

## Scope and defaults

Only the Bullet profile enables the floor. Guidance uses visual bearing, attitude, and barometric altitude, never simulator target height or physics pose.

## Status

Implemented; 113 App tests, 20 focused LOS/estimation tests, and SITL target contact passed. Implementation commit: Pending (no commit requested).
