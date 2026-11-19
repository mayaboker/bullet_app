# LOS sensor tick staleness

Goal: stop fresh MAVLink sensor streams from appearing stale to LOS when their measurements straddle a fixed estimator tick, and explain the observed failure.

Implementation: select measurements on both sides of a tick and interpolate when the latest measurement is ahead of that tick. Retain the bounded prediction and source-age limits. Count barometer interpolation and missing ticks, report them at the existing one-second LOS diagnostic cadence, and log an LOS fault once at WARNING with source and navigator ages. Also report ATTITUDE clock-mapping age and rejections so a separate TIMESYNC issue is visible.

Interfaces: no CLI, MAVLink, or LOS protocol changes. Estimator diagnostic counters are internal to Bullet App.

Verification: reproduce `barometer_stale` with steady 47 Hz barometer and control timing, verify the corrected estimator does not fault, run the App tests, and run the automatic target mission against the active SITL stack. Stop the App after the flight and write a findings report.

Scope/defaults: preserve the 60 ms prediction limit and 200 ms LOS freshness limit. The fallback uses only measurements already received by the App. No commit requested.

Status: Implemented. The regression failed before the fix; all 30 App tests pass afterward. Two initial live LOS runs had zero missing barometer ticks and no `barometer_stale`, then landed and disarmed. After the SITL shared-clock patch, the target mission reached commit, reported a hit, and disarmed with zero ATTITUDE mapping rejections; see [the report](../docs/los-barometer-stale-report.md). Implementation commit: Pending (no commit requested).
