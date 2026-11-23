# GPS MAVLink streams for MCAP

## Goal

Request GPS_RAW_INT and GLOBAL_POSITION_INT from Betaflight when configured for publication, and record their blackboard snapshots at 2 Hz in the workspace mission stacks.

## Implementation steps

1. Request each enabled GPS message at its configured publication rate during MAVLink setup.
2. Set both workspace launch profiles to publish both GPS topics at 2 Hz.
3. Enable the launcher virtual-GPS path in both workspace stacks so Betaflight produces samples.
4. Update App, simulator, and workspace README guidance.

## Interface changes

No new topic or wire format. Existing `telemetry/gps_raw_int` and `telemetry/global_position_int` channels are used.

## Scope and defaults

The packaged App configurations keep GPS publishing disabled by default. The workspace SITL launch profiles enable both streams at 2 Hz using virtual GPS.

## Verification

Passed: 21 focused App tests, both workspace YAML rate checks, and GPS-capable SITL binary/profile validation. A live check was not run because ports 5761 and 5762 were already occupied.

## Status

Implemented. Implementation commit: Pending (no commit requested).
