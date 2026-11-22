# Record raw MAVLink telemetry through ZMQ and MCAP

Goal: Republish the vehicle MAVLink packets that Bullet App currently uses so `mcap_bagger` can record their exact bytes and the App's publication time without decoding MAVLink.

Implementation steps:
1. Add an optional `--telemetry-pub-endpoint` to Bullet App, default disabled; use `tcp://127.0.0.1:5557` in the documented recording command. Start a dedicated ZMQ PUB worker before connecting to MAVLink.
2. Add an optional observation callback to `beta_client.MavlinkTelemetry` that receives source-filtered original wire bytes for incoming `HEARTBEAT`, `ATTITUDE`, `VFR_HUD`, and, when the target mission enables it, `TIMESYNC`. Keep the callback independent of ZMQ and the flight loop.
3. Pass accepted packets to a bounded, nonblocking App publisher queue. The publisher sends `[topic, header, payload]`, with topics `telemetry/heartbeat`, `telemetry/attitude`, `telemetry/vfr_hud`, and `telemetry/timesync`, an eight-byte little-endian unsigned UTC Unix nanosecond header stamped immediately before sending, and unchanged MAVLink wire bytes as payload. Count and warn about local queue/send drops; recording backpressure does not stop flight.
4. Extend `mcap_bagger` to use the header as MCAP `publish_time`, local receive time as `log_time`, and the payload unchanged as MCAP data. Keep legacy two-frame `[topic, payload]` input. Update the bagger's sample channel configuration and both projects' usage documentation.

Interface changes: `--telemetry-pub-endpoint` enables publication; the three-frame ZMQ format is `[UTF-8 topic, struct.pack('<Q', time.time_ns()), raw MAVLink packet]`. Bagger validates only framing and timestamp, never MAVLink contents. Legacy two-frame messages continue to use receive time for both MCAP timestamps.

Verification: Test exact packet-byte and topic preservation, source filtering, all four selected message types, timestamp header encoding, drop handling, malformed headers, and legacy two-frame compatibility. Run beta_client, App, and bagger suites; check a live SITL packet through ZMQ into MCAP and read back `publish_time`, `log_time`, and data.

Scope/defaults: Publication is opt-in and uses the existing loopback sample endpoint. `time.time_ns()` at ZMQ publication is the publish timestamp; Betaflight `time_boot_ms` remains inside the raw packet. GPS and command acknowledgments are out of scope. Start the bagger before App to reduce PUB/SUB startup loss. No code, configuration, or runtime behavior has been changed by saving this plan.

Status: Planned. Implementation commit: Pending.
