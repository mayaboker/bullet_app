# Manual workspace flight and viewer recording controls

- Status: Implemented; verified 2026-11-02
- Implementation commit: `41ce23a7fbbc08f5e63f7b65faf6bb1a6ada9f72`

Configure the Run all launcher through a source-controlled operator config:
manual Arm/Start and MAVLink recording. Keep standalone defaults and the existing
Godot bringup example. Add persistent Start/Stop recording buttons in Bullet View,
worker-owned command handling, addressed ACK confirmation, actual recorder state
and authority telemetry, stale/timeout/error handling. Recording stays independent
of flight. Preserve existing work and do not commit or push.

Verify launcher arguments/config validation, widget/presenter routing, command
ACK matching, remote state, rejection/timeouts and existing flight/recording tests.
Use fake/internal backends only; no camera or motor actuation.

## Result

Run all reads config/operator_controls.yaml (manual arming, MAVLink recording),
forwards settings to bullet_app and selects the recorder's recording control mode.
Direct legacy launcher calls without the new config retain automatic/Godot.
Production bringup already selects manual/MAVLink and retains detection-only
behavior unless a compatible mission profile and fresh flight feedback are given.

Bullet View has persistent recording Start/Stop controls, immutable remote state,
worker-owned commands and presenter routing. ACK source, destination, command and
action are matched. Stop supersedes Start; late Start ACKs cannot complete Stop.
Success requires both ACK and actual remote state; timeout/stale/disconnect,
Godot authority and recorder error remain explicit. Actual recording state is
forwarded independently of flight via REC_AUTH/REC_STATE. No automatic retries.

Verification: workspace 15, bullet_app 196, bullet_view 40, bullet_msgs 182,
bullet_bringup 21 = 454 tests passed with uv and the shared lockfile. Real MAVLink
UDP→application→ZMQ→recorder Start/Stop produced finalized readable MCAPs and
confirmed action-specific ACKs and named state. Qt panels were rendered and
visually inspected. A Qt shutdown failure was addressed by retaining one session
QApplication through interpreter shutdown; the offscreen viewer suite then passed
three consecutive runs. Shell syntax, launcher help and Git whitespace checks
passed. No real camera/motors were actuated, no running session was restarted,
and no commits or pushes were made. Restart Run all to activate its settings.
Implementation commit: `41ce23a7fbbc08f5e63f7b65faf6bb1a6ada9f72`.

## Commit record — 2026-11-02

The verification notes above describe the pre-commit work. Implementation is
now committed; nothing was pushed. These are implementation hashes, rather than
the subsequent documentation commits that record them.

| Repository | Implementation commit |
|---|---|
| bullet_app | `41ce23a7fbbc08f5e63f7b65faf6bb1a6ada9f72` |
