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
| [0029 Replace GitHub main](0029-replace-github-main.md) | Refresh the lockfile and publish this checkout while backing up the unrelated GitHub history. | Published and verified | `c0dcd83ccdeeb4fb48dceecd07f6fa291f6a6263` |
| [0030 Selectable target mission](0030-selectable-target-mission.md) | Add startup-selected red-truck search and MAVLink/bbox LOS guidance. | Pending; focused tests passed, live SITL check blocked by occupied port | Pending |
| [0031 VS Code mission choice](0031-vscode-mission-choice.md) | Choose hover-land or target-hit when running the automatic App task. | Pending; task and CLI values verified | Pending |
| [0032 Mission package and approval modes](0032-mission-package-and-approval-modes.md) | Separate missions, configure hover/land, and use manual/auto approval for either mission. | Implemented; tests and SITL smoke passed | `d4fae85bf7d75faeefb01fcf5438a3b0385949db` |
| [0033 View bbox extension](0033-view-bbox-extension.md) | Forward validated fid-tagged detections to View through MAVLink. | Implemented; tests and live smoke passed | `ef32619903337bf9baa67b593b3fe29b51509007` |
| [0034 Telemetry rate and LOS timing](0034-telemetry-rate-and-los-timing.md) | Report upstream rates and image/attitude alignment before changing LOS buffering. | Implemented; tests and live diagnosis passed | `9cfa2de` (App), `069f3b8` (LOS) |
| [0035 Fixed-rate LOS estimator](0035-fixed-rate-los-estimator.md) | Smooth and resample target-hit sensors while preserving camera capture times. | Implemented; tests and live target hit passed | `9cfa2de` |
| [0036 MAVLink TIMESYNC](0036-mavlink-timesync.md) | Align SITL attitude with the camera clock and gate target flight on fresh sync. | Implemented; tests and live target hit passed | `5e5ba45` (beta_client), `9960df8` (App) |
| [0037 Colored terminal logs](0037-colored-terminal-logs.md) | Color Bullet App terminal diagnostics while keeping redirected logs readable. | Implemented; output verified | Pending (no commit requested) |
| [0038 LOS sensor tick staleness](0038-los-sensor-tick-staleness.md) | Keep fresh MAVLink samples available across estimator ticks and diagnose LOS faults. | Implemented; tests and live barometer check passed | `cd22cd8` |
| [0039 Manual target settings](0039-manual-target-settings.md) | Interim two-value target settings design. | Superseded by 0040 | `cd22cd8` |
| [0040 Height-only target control](0040-height-only-target-control.md) | Restore one target height setting and fix pre-search hover at five seconds. | Implemented; 32 App tests passed | `cd22cd8` |
| [0041 Persistent tracking logs](0041-persistent-tracking-logs.md) | Save each App run and 5 Hz target guidance checkpoints in workspace logs. | Implemented; 36 App tests and CLI smoke passed | `3b40552` (App), `e81c4db` (workspace) |
| [0042 SITL identification in App heartbeat](0042-sitl-heartbeat.md) | Query MSP board identity at startup and carry SITL confirmation in App heartbeat. | Implemented; tests and live SITL heartbeat passed | `3fbc039dd13634274c1896758f830876c598b8c9` (beta_client), `ed4d66aa2c2c392e612a9f8529ed32e6b81d371e` (App) |
| [0043 Raw MAVLink ZMQ/MCAP recording](0043-mavlink-zmq-mcap-recording.md) | Earlier raw-packet approach replaced by blackboard MsgPack publication in 0063. | Superseded | Pending |
| [0044 Mission control loop diagram](0044-mission-control-loop-diagram.md) | Explain App components and its 50 Hz loop with Mermaid diagrams. | Implemented; links and source checked | `b6fd6f7`, `fae5430` (follow-up) |
| [0045 Hover-land mission guide](0045-hover-land-mission-guide.md) | Explain the implemented hover-land mission, control thresholds, and operator behavior. | Implemented; links and source checked | `b6fd6f7`, `fae5430` (follow-up) |
| [0046 Transitions mission state machine](0046-transitions-mission-state-machine.md) | Move shared and target mission phases into an ordered transitions machine. | Implemented; 42 tests passed | `b6fd6f7`, `fae5430` (follow-up) |
| [0047 Mission blackboard and step context](0047-mission-blackboard-context.md) | Share current MAVLink, visual, and target feature data through a singleton board and step context. | Implemented; 45 tests passed | `fae5430` |
| [0048 Target-hit mission guide](0048-target-hit-mission-guide.md) | Explain target-hit setup, phase transitions, LOS inputs, contact, and failure behavior. | Implemented; links and source checked | `fae5430` |
| [0049 Shared mission base](0049-shared-mission-base.md) | Keep the base mission limited to shared flight behavior and move target-only methods to concrete missions. | Implemented; 51 tests passed | `fae5430` |
| [0050 Local readiness enum](0050-local-readiness-enum.md) | Name App readiness values while preserving numeric MAVLink status and avoiding a shared Python flight contract. | Implemented; 52 App and 90 View tests passed | `fae5430` |
| [0051 Mission action enum](0051-mission-action-enum.md) | Use App-owned action members for mission requests while preserving MAVLink commands and ACK IDs. | Implemented; 53 tests passed | Pending (no commit requested) |
| [0052 Mission mode enum](0052-mission-mode-enum.md) | Store approval mode as an App-owned enum while keeping CLI text and MAVLink status values. | Implemented; 54 tests passed | Pending (no commit requested) |
| [0053 StepContext mission chain](0053-step-context-mission-chain.md) | Pass one StepContext through shared mission tick helpers and hover-completion hooks. | Implemented; 54 tests passed | Pending (no commit requested) |
| [0054 Rename mission runtime to app](0054-rename-mission-runtime-to-app.md) | Rename the runtime module and update imports and documentation references. | Implemented; 54 tests and CLI help passed | Pending (no commit requested) |
| [0055 KISS method readability guidance](0055-kiss-method-readability.md) | Guide extraction of cohesive helpers from long orchestration methods without needless abstraction. | Implemented; diff checked | Pending (no commit requested) |
| [0056 Readable App control loop](0056-readable-app-control-loop.md) | Extract input, safety, and diagnostics responsibilities from the 50 Hz runtime loop. | Implemented; 57 tests passed | Pending (no commit requested) |
| [0057 Betaflight connection config](0057-betaflight-connection-config.md) | Select TCP or serial MSP and MAVLink links from a commented YAML config. | Implemented; 65 tests and wheel build passed | Pending (no commit requested) |
| [0058 Open MAVLink helper](0058-open-mavlink-helper.md) | Make TCP and serial MAVLink construction explicit beside the MSP helper. | Implemented; 65 tests passed | Pending (no commit requested) |
| [0059 Target tracking-log ownership](0059-target-tracking-log-ownership.md) | Move tracking-log state into TargetMission and observe commands after MSP send. | Implemented; 68 tests passed | Pending (no commit requested) |
| [0060 Separate tracker and simulation inputs](0060-separate-tracker-simulation.md) | Split tracker boxes from optional simulator contact and gate contact through YAML. | Implemented; 77 tests passed | Pending (no commit requested) |
| [0061 Independent input openers](0061-independent-input-openers.md) | Give tracker and optional simulation contact separate App setup paths. | Implemented; 78 tests passed | Pending (no commit requested) |
| [0062 Colored mission state logs](0062-colored-mission-state-logs.md) | Log each accepted mission phase change with terminal color. | Implemented; 79 tests passed | Pending (no commit requested) |
| [0063 Blackboard MCAP publisher](0063-blackboard-mcap-publisher.md) | Publish configured blackboard values through an isolated MsgPack/ZMQ worker. | Implemented; 96 App and 78 bagger tests passed | Pending (no commit requested) |
| [0064 Telemetry publisher port](0064-telemetry-publisher-port.md) | Move the App publisher off renderer control port 5557 and align recorder subscriptions. | Implemented; 26 App and 18 Bagger tests passed | Pending (no commit requested) |
