# Open MAVLink helper

Goal: Make the TCP/serial MAVLink startup selection explicit beside the MSP transport helper.

Implementation steps: Add `_open_mavlink(config)` in App, returning `MavlinkTelemetry` for the configured TCP endpoint or serial device and baud rate. Enter its context in `run_mission`, preserving rate requests, TIMESYNC setup, and resource order. Update focused startup tests for both modes.

Interface changes: Internal helper only; YAML, CLI, and MAVLink behavior remain unchanged. TCP uses the receiver's default baud argument, which does not affect a TCP socket; serial passes configured baud rate.

Verification: The locked App suite passed (65 tests), including runtime startup checks that TCP uses its endpoint and serial uses its device and configured baud rate. `git diff --check` passed.

Scope/defaults: No new transport class or protocol implementation. Preserve the uncommitted connection-config changes from plan 0057.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
