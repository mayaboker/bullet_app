# Blackboard telemetry publication

Bullet App can publish selected current blackboard values to `mcap_bagger` without encoding or ZMQ work in the 50 Hz mission loop. The worker thread reads the latest value at each configured topic rate. A positive FPS repeats the latest available value even if it has not changed; `0` disables that topic. Missing `None` values are skipped, while `false` is published. Rates are limited to 50 FPS. An absent `telemetry_publisher` section or all-zero rates starts no worker or socket.

Copy [App's commented YAML](../src/bullet_app/app.yaml), set `telemetry_publisher.topics` rates, and pass the copy with `--config`. The endpoint must be loopback TCP and must match the ordinary channels in [bagger's example config](../../mcap_bagger/config/recorder.yaml). Start the bagger before App to reduce PUB/SUB startup loss. Publication is best effort; a subscriber can miss startup or overloaded messages.

For the GPS topics, a positive configured rate also requests `GPS_RAW_INT` or `GLOBAL_POSITION_INT` over MAVLink at that rate during App startup. If the firmware has no configured GPS source, the blackboard values remain absent and the worker skips those topics. The workspace launch profiles configure both at 2 Hz and enable virtual GPS in SITL.

| Topic | Blackboard value | Additional payload fields |
| --- | --- | --- |
| `telemetry/heartbeat` | `telemetry.heartbeat` | `received_at_s` |
| `telemetry/attitude` | `telemetry.attitude` | `received_at_s`, `rate_hz` |
| `telemetry/vfr_hud` | `telemetry.vfr_hud` | `received_at_s`, `rate_hz` |
| `telemetry/gps_raw_int` | `telemetry.gps_raw_int` | `received_at_s`, `rate_hz` |
| `telemetry/global_position_int` | `telemetry.global_position_int` | `received_at_s`, `rate_hz` |
| `telemetry/timesync` | `telemetry.timesync` | `received_at_s` from the estimate update |
| `visual/bbox` | `visual.bbox` | — |
| `simulation/target_contact` | `simulation.target_contact` | — |
| `features/navigation_input` | `features.navigation_input` | — |
| `features/navigation_result` | `features.navigation_result` | — |
| `view/alive` | `view_alive` | — |

Each ZMQ message has three frames: `[UTF-8 topic, struct.pack('<Q', publish_time_ns), MsgPack payload]`. The eight-byte header is an unsigned Unix epoch nanosecond timestamp. The MsgPack payload is a map with `schema_version: 1`, `value`, and `timestamp_source` (`mavlink`, `receive`, or `publish`), plus the listed metadata. Nested dataclasses become maps and enums become their values. Bagger stores the payload bytes unchanged, uses the header as MCAP `publish_time`, and uses its own receive clock as MCAP `log_time`.

For ATTITUDE and GLOBAL_POSITION_INT, source `time_boot_ms` is converted to Unix time with fresh MAVLink TIMESYNC and the worker's host monotonic-to-wall-clock bridge. GPS_RAW_INT `time_usec` uses a Unix timestamp directly when it has the Unix range; otherwise it is treated as boot time and converted the same way. When source conversion is unavailable, the header uses the telemetry value's local receive time mapped to Unix time. HEARTBEAT and VFR_HUD have no source timestamp and use that receive-time fallback. Other blackboard values use worker publication time; their original timestamps, if present, remain in `value`. Repeated source or receive values retain the same header timestamp.

The worker owns the PUB socket, MsgPack encoding, and scheduling. It uses nonblocking sends and skips missed deadlines rather than sending a burst. Invalid configuration or inability to bind the endpoint stops App before flight; runtime publisher errors stop recording and log an error without faulting the mission. The worker shares process CPU with App, so select rates appropriate for the host.
