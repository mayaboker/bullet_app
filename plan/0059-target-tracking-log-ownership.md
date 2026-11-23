# Target tracking-log ownership

Goal: Let `TargetMission` own target-specific tracking diagnostics while preserving logs of the actual RC command sent by App.

Implementation steps: Create `TrackingLog` in `TargetMission`. Add a shared post-send hook to `MissionBase` and override it in `TargetMission` to observe the context and command. Call the hook in App immediately after `_send_command`; remove target-specific logger construction and branching from the global loop. Import `Phase` directly from `missions.base` in `tracking_log.py` to avoid an import cycle.

Interface changes: Add `MissionBase.on_command_sent(context, command) -> None` as a mission lifecycle hook. No CLI, protocol, log format, or send-order changes.

Verification: The locked App suite passed (68 tests). Tests cover target ownership, the hover no-op, delivery of normal and safe fallback commands to the post-send hook, bounded tracking checkpoints, and App send/publish ordering. `git diff --check` and documentation links passed. The root README remains accurate.

Scope/defaults: Keep the existing TrackingLog class and its format. Do not log before the MSP send. Preserve unrelated working-tree changes.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
