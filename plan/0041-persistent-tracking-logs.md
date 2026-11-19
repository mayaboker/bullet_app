# Persistent tracking logs

Goal: save each Bullet App run to a timestamped file in `~/workspaces/bullet_ng/logs` and expose enough tracking checkpoints to diagnose missed targets.

Implementation: configure terminal INFO and file DEBUG Loguru sinks at the App entry point, fail startup if the file cannot be opened, and emit 5 Hz file-only checkpoints during target search and LOS. Log target visibility, navigation stage, contact, and failure events once per transition. Ignore generated logs in the workspace repository and document their location and contents.

Interfaces: log filename `bullet_app_<local timestamp>.log`; no MAVLink, CLI, or mission-control changes. Checkpoints use existing bbox, LOS diagnostics, and sent RC values, without claiming a measured horizontal position.

Verification: log path and sink tests, checkpoint cadence and missing-field tests, event-transition tests, App suite, and workspace Git ignore check.

Scope/defaults: one file per App process, no automatic retention cleanup; terminal remains INFO, file captures DEBUG. If file creation fails, the mission does not start. No commit requested.

Status: Implemented; 36 App tests passed, CLI startup created a log in the requested directory, and the workspace ignore rule matched. Implementation commits: `3b40552` (App), `e81c4db` (workspace).
