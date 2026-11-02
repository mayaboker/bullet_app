# MSP parameter queue diagnostics schema repair

- Status: Implemented; verified 2026-11-02
- Implementation commit: Pending

Fix recorder rejection of analysis/app/msp sensor_feedback fields
output_parameters_queue_depth and output_dropped_parameters. Extend the shared
strict schema with optional nonnegative integer counters, preserving old packets
and rejecting unknown fields. Test real runtime publication diagnostics against
the schema and finalized MCAP readback with both counters. Do not alter arming,
recording authority or lifecycle policy; preserve existing work, no commit/push.

## Result and verification

The shared analysis_msp sensor_feedback schema now accepts optional nonnegative
integer output_parameters_queue_depth and output_dropped_parameters. Existing
required fields, schema version and additionalProperties=false remain unchanged.
Old packets still validate; unknown keys, negative counters, fractional/string/
boolean counters remain rejected.

The application test derives publication diagnostic keys from a real runtime
blackboard including parameter-queue overflow, so a future producer/schema drift
fails the test. The recorder test starts new recordings in ARMED_IDLE and
LOS_ACQUIRE, writes the counters, finalizes MCAP without error/partial output,
and reads back the exact packet with CRC validation.

285 unique tests passed: bullet_msgs 185; bullet_app runtime tests 7; mcap_bagger
93 (92-test full suite before adding the ARMED_IDLE parameterized case, followed
by the six-case analysis recording suite covering both phases). Whitespace checks
passed. No flight/recording authority/lifecycle behavior changed, and no physical
motors were actuated. Restart the workspace launcher/recorder before a new run:
recorders cache their schema validators in startup configuration. No running
session was restarted; no commits or pushes. Implementation commit: Pending.
