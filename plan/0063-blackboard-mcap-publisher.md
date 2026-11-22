# Blackboard MCAP publisher

Goal: Publish configured blackboard values at independent rates to `mcap_bagger` without work in the mission tick.

Implementation steps: Add optional YAML topic rates and a background ZMQ PUB worker. Serialize values as MsgPack, send topic plus Unix nanosecond header plus opaque payload, and normalize MAVLink source times when available. Enable TIMESYNC for active publishing. Extend bagger to accept the three-frame timestamp form while retaining legacy input. Update examples, docs, and tests.

Interface changes: `telemetry_publisher.endpoint` and `telemetry_publisher.topics` in App YAML. Fixed topic catalog; per-topic FPS 0–50, with zero disabled. Three-frame ZMQ `[topic, <Q timestamp, msgpack payload]`; bagger uses header as MCAP publish time and receipt as log time.

Verification: App suite passed (96 tests); bagger suite passed (78 tests). Tests cover config, per-topic worker output, repeated timestamps, source/receive clock selection, bind and send failures, legacy and malformed bagger framing, and MCAP time/data preservation. Both wheels built, App docs links and diff check passed. A live two-process App-to-bagger smoke run recorded ten `view/alive` messages with distinct MCAP publish/log times.

Scope/defaults: Publisher is opt-in. The worker reads the blackboard directly; no encoding or ZMQ work in the 50 Hz loop. Send failures do not fault flight. This supersedes raw-wire plan 0043.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
