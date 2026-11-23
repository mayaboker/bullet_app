# Record MSP RC commands in MCAP

Goal: Capture each successfully sent MSP RC frame with its send timestamp for pitch-response analysis.

Implementation steps: Add an event-enabled, bounded command queue to the existing telemetry publisher. Enqueue the exact eight RC channels after each successful send, including final disarm frames. Publish `control/msp_rc` with the existing ZMQ header and MsgPack envelope. Enable the stream in the target-hit SITL profile, register the Bagger channel, and decode it in FlightLens's MSP tab.

Interface changes: Add `telemetry_publisher.record_msp_rc` (default false). The `control/msp_rc` value contains a sequence number, eight PWM channels, mission phase, and monotonic send time; the header is send time converted to Unix nanoseconds.

Verification: App's 105 tests, Bagger's 86 tests, and FlightLens's 124 tests passed. They cover config validation, event-only startup, payload ordering and timestamps, bounded drops, shutdown delivery, MCAP publish-time readback, and FlightLens UI decoding. Diff checks passed.

Scope/defaults: Keep the sampled blackboard topics unchanged. Record successful transport sends, not firmware acknowledgments. Delivery remains best effort and never blocks the control loop.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
