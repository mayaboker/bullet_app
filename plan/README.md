# Plans

| Plan | Description | Status | Implementation commit |
| --- | --- | --- | --- |
| [0023 Initialize workspace Git repositories](0023-initialize-workspace-git.md) | Create separate local repositories for the four workspace folders. | Implemented and verified | Pending |
| [0001 Bullet App SOLID + CLI Refactor](0001-bullet-app-solid-cli-refactor.md) | Separate Click parsing and application/runtime orchestration while preserving the CLI. | Implemented | `ffc057d` (`5c3f221` lockfile) |
| [0002 MSP RC Scheduler](0002-msp-rc-scheduler.md) | Stream and record latest MSP RC delivery at 50 Hz with a firmware-compatible safe fallback. | Pending | Pending |
| [0003 Betaflight Takeoff Tuning](0003-betaflight-takeoff-tuning.md) | Calibrate safe Betaflight takeoff and hover PWM in simulation. | Pending | Pending |
| [0004 MSP Telemetry Scheduler](0004-msp-telemetry-scheduler.md) | Schedule MSP status, attitude, and altitude polling with cached snapshots. | Implemented | `af360c7` |
| [0005 Runtime Blackboard](0005-runtime-blackboard.md) | Isolate ZMQ reception from mission control with stamped shared runtime state. | Implemented | `c95784a` |
| [0006 Runtime Publisher](0006-runtime-publisher.md) | Isolate ZMQ publication from mission control with scheduled shared output state. | Implemented | `0e440ed` |
| [0007 MSP Analysis Schema Compatibility](0007-msp-analysis-schema-compatibility.md) | Keep current MSP diagnostics compatible with strict MCAP recording schemas. | Implemented | `2aa461a` (`bullet_msgs`), `4f2be3e` (`mcap_bagger`) |
| [0008 MAVLink Telemetry Adapter](0008-mavlink-telemetry-adapter.md) | Publish isolated MAVLink heartbeat, attitude, and altitude from mission feedback. | Implemented | `76aa876` |
| [0009 Configurable yaw search timeout](0009-configurable-yaw-search-timeout.md) | Expose and extend the yaw-search deadline through navigator tuning. | Implemented | `6ace504`, `93cb08d`, `f3d0d74` |
| [0010 Heavy quad mission profiles](0010-heavy-quad-mission-profiles.md) | Align mission vehicle limits with the selected heavy_quad physics profile. | Implemented | `32adb71` (`ecf9300` workspace profiles) |
| [0011 Vertical LOS centering](0011-vertical-los-centering.md) | Keep the target on the image Y centerline and brake excessive descent with thrust. | Implemented; tests passed | `7a76c43`, `acb0b5d` |
| [0012 Aggressive X-axis tracking tuning](0012-aggressive-x-axis-tuning.md) | Increase heavy-quad lateral tracking authority after roll saturation in the latest flight. | Pending | Pending |
| [0013 Dedicated aggressive takeoff controller](0013-dedicated-takeoff-controller.md) | Use an application-owned vertical PID for a 4 m/s heavy-quad takeoff. | Implemented; tests passed | `89049df`, `4fe4147`, `84a11d0` |
| [0014 Stop-only mission profile reload](0014-stop-only-profile-reload.md) | Reload the mission YAML from Bullet View while the scenario is stopped. | Pending | Pending |
| [0015 Wind-robust optical LOS recovery](0015-optical-recovery-wind-braking.md) | Preserve bounded optical roll and vertical corrections during visual recovery under wind. | Implemented; tests passed | `abc7814` (`los_tracking`) |
| [0016 Bullet Perception Library](0016-bullet-perception-library.md) | Extract ndarray-to-bounding-box detection into a reusable workspace package. | Pending | Pending |
| [0017 Yaw overshoot tuning](0017-yaw-overshoot-tuning.md) | Reduce heavy-quad Betaflight first-approach yaw overshoot through a lower proportional gain. | Implemented | `3db6fbf` |
| [0018 Configurable Betaflight connection](0018-configurable-betaflight-connection.md) | Configure Betaflight MSP TCP or serial control from the mission YAML, independently of MAVLink. | Implemented | `bb42c55` (`ef75583`, `05a0076`) |
| [0019 MAVLink AGL telemetry](0019-mavlink-agl.md) | Publish physics rangefinder AGL over DISTANCE_SENSOR and retain it in flight feedback/MCAP. | Implemented; focused tests passed | `7a5774f` |
| [0020 LOS roll alignment](0020-los-roll-alignment.md) | Preserve yaw-first centering and verify roll correction during LOS tracking. | Implemented; tests passed | `d58580a` |
| [0021 TTC stability gating](0021-ttc-stability-gating.md) | Gate noisy TTC estimates before descent-feasibility control. | Implemented; tests passed | `504fc99`, `4241615`, `744968a` |
| [0022-recorded-issue-triggers](0022-recorded-issue-triggers.md) | Explain tracking triggers from recorded measurements and limits. | Pending | Pending |
| [0024 Standalone auto mission](0024-standalone-auto-mission.md) | Run a separate MSP/MAVLink app for 10 m takeoff, 10 s hover, landing and disarm. | Implemented; verified in SITL | Pending (no commit requested) |
| [0025 VS Code auto mission task](0025-vscode-auto-mission-task.md) | Run the standalone app from VS Code after starting the launcher. | Implemented; task verified | Pending (no commit requested) |
| [0026 View-gated Auto and Manual flight](0026-view-gated-flight.md) | Wait for View Arm/Start; support manual height, Land and repeated flights. | Implemented; verified in SITL | Pending (no commit requested) |
| [0027 Manual VS Code app task](0027-manual-vscode-task.md) | Start Bullet App in Manual mode from the single VS Code task. | Implemented; task verified | Pending (no commit requested) |
| [0028 Single-port Bullet View link](0028-single-port-view-link.md) | Send telemetry, receive View commands, and return ACKs through one UDP socket. | Implemented and verified | Pending (no commit requested) |
| [0029 Replace GitHub main](0029-replace-github-main.md) | Refresh the lockfile and publish this checkout while backing up the unrelated GitHub history. | Pending | Pending |
