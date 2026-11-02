# Manual flight controls

- Status: Implemented
- Implementation commit: Pending

Implement workspace `plan/0006-manual-flight-controls.md`: separate Arm and Start,
confirmed takeoff-height override, asynchronous MAVLink commands, fresh observed
arming state, persistent disarm latch, and an MVP flight panel. Preserve automatic
simulation defaults and existing recording/parameter commands. Verify with fake
and internal backends only; no physical motor actuation.

Verification results will be recorded on completion.

Verification: 195 project tests passed with uv/shared lockfile; see workspace plan 0006 for cross-project results, installation checks and limitations. Implementation commit: Pending.
