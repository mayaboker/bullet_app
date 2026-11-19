# LOS `barometer_stale` investigation

Date: 2026-11-19. Mission: `target-hit` against the running Betaflight SITL stack.

## Observed failure

The stopped App's manual-flight log entered LOS at 13:40:54 and then landed with `LOS barometer_stale`. VFR_HUD was arriving at about 47 Hz, and its latest sample was 9 ms old. The LOS profile allows 200 ms of barometer age, so the upstream stream rate did not explain the fault.

The estimator advances on a 20 ms grid. Its previous implementation inspected only the newest barometer measurement. If that measurement was received after the grid tick, the estimator returned no barometer, even when the preceding measurement covered the tick. LOS retained its last estimated sample until its age exceeded 200 ms. A deterministic 47 Hz input reproduced `barometer_stale` at 2.244 s with an incoming sample only 0.2 ms old and the last delivered estimate 213 ms old.

## Change and diagnostics

The [sensor estimator](../src/bullet_app/sensor_estimator.py) now interpolates between received measurements when the newest one lies after the tick. It applies this to barometer and attitude, and keeps the existing 60 ms source prediction limit. It counts barometer interpolation and missing ticks.

The [App loop](../src/bullet_app/mission.py) logs those counters at INFO once per second during LOS. An LOS fault is logged once at WARNING with the raw source ages, navigator ages, tick lag, interpolation count, and missing count. The [target mission](../src/bullet_app/missions/target_hit.py) also reports ATTITUDE clock-mapping age and rejection count. Normal LOS state changes remain at INFO; there is no per-sample warning flood.

## Verification and clock follow-up

The regression test first failed on the old estimator with `barometer_stale` while the raw sample was fresh. With the fix, all 30 App tests pass. Two automatic live flights after the estimator change showed **zero missing barometer ticks** during LOS; the first recorded 92 interpolated ticks and the final run recorded 13. Neither flight reported `barometer_stale`. Both landed and disarmed, and the App was stopped afterward.

Those two flights did not complete the target mission. They failed with `LOS acquisition_timeout` because no ATTITUDE sample passed the TIMESYNC mapping check. Mapped ATTITUDE age was about **-22 to -38 ms**, meaning the mapped timestamp was in the future, while VFR_HUD remained fresh. The old SITL firmware answered TIMESYNC using `micros()` and stamped ATTITUDE using `millis()`; SITL advanced those clocks in separate scaled accumulators.

Follow-up: the Bullet Drone GPS SITL patch now derives `millis64()` from `micros64()` and answers TIMESYNC with the 64-bit microsecond clock. After rebuilding and restarting SITL, 429 live ATTITUDE samples mapped to ages of **5.8–26.8 ms**, with none in the future. A subsequent automatic target flight reached LOS commit, reported a confirmed hit, disarmed, and exited successfully. It recorded zero ATTITUDE mapping rejections and zero missing barometer ticks during LOS. Bullet App was stopped after the flight.
