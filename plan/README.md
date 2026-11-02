# Plans

[0026 Recording control](0026-recording-control.md) — Implemented; 173 application tests and installed roundtrip passed; commit Pending.

| Plan | Description | Status | Implementation commit |
| --- | --- | --- | --- |
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
| [0022 Runtime parameters](0022-runtime-parameters.md) | MAVLink parameter UI, live tuning and remote update logs. | Implemented; tests passed | `cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d` |
| [0023 Status time and frame](0023-status-time-frame.md) | Show simulation seconds and camera sequence in status logs. | Implemented; 361 tests passed | `cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d` |
| [0024 Recorded file origins](0024-recorded-profile-origins.md) | Save source profile/config filenames and show only application parameters in FlightLens. | Implemented; verification passed | `cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d` |
| [0025 Heavy-quad descent limit](0025-heavy-quad-descent-limit.md) | Set both heavy-quad mission profiles to 5.5 m/s maximum commanded descent. | Implemented; profile loads and unchanged-value checks passed | `8c47f8feecf28acc718d3dff57e24a779318b124` (workspace) |

- [0027: Manual flight controls](0027-manual-flight-controls.md) — Implemented; 195 tests passed; commit Pending.

- [0028: Manual workspace and recording controls](0028-recording-viewer-state.md) — Implemented; 196 tests passed; commit Pending.

- [0029: Mission collision gates](0029-mission-collision-gates.md) — Implemented; 197 tests passed; commit Pending.

- [0030: MSP diagnostic schema repair](0030-msp-parameter-diagnostics-schema.md) — Implemented; 7 tests passed; commit Pending.
