# MAVLink TIMESYNC for target guidance

Goal: align the SITL boot clock used by ATTITUDE with the App monotonic clock used by camera capture and LOS, and refuse target flight when that alignment is unavailable.

Implementation: add an opt-in, worker-owned TIMESYNC exchange to beta_client at 5 Hz. Accept replies only from the selected vehicle that echo the outstanding request and arrive within 40 ms. Use a median of recent clock offsets, require five valid replies, and mark sync stale after one second. Map ATTITUDE.time_boot_ms through the accepted offset. The target mission waits for sync before Arm/Start, faults after five seconds without startup sync, disarms if sync is lost while armed idle, and lands if lost in flight. Log sync quality and transitions.

Interfaces: add immutable sync status to TelemetrySnapshot and an enable_timesync method. Hover-land remains unchanged. View readiness and command rejection reflect target sync availability.

Verification: beta_client worker/filter/timeout tests; App mission gating, timestamp mapping, and loss tests; full App and beta_client suites; live SITL target-stack run.

Scope/defaults: local commits only. TIMESYNC is target-hit opt-in. Request period 0.2 s, maximum accepted RTT 0.04 s, minimum five replies, stale after 1 s, startup wait 5 s.

Status: Implemented. beta_client and App tests passed. A live SITL target mission acquired sync, completed LOS, and reported a confirmed hit; the simulator and Visual process were stopped afterward. Implementation commits: `5e5ba45` (beta_client), `9960df8` (App).
