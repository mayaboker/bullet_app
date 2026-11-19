# Mission package and approval modes

Status: Pending
Implementation commit: Pending

Goal: Separate shared flight behavior from hover/land and target-hit missions. Make hover/land use a startup YAML profile with 20 m and 10 s defaults, and make manual/auto an approval choice for either mission.

Implementation: Create `bullet_app.missions` with shared flight transitions, mission-specific modules, config loading, and a factory. Manual mode waits for View Arm/Start; auto mode starts after fresh SITL feedback, runs without View, and exits after one confirmed disarm. Move target guidance into its own module and remove only its preflight Visual gate. Keep mission selection at startup.

Interfaces: `--mission hover-land|target-hit`, `--mode manual|auto` (manual default), and `--mission-config PATH` with a per-mission default. View shows configured height read-only and gates Arm/Start only in manual approval mode. Update VS Code tasks and the workspace stack to use manual approval.

Verification: Unit tests for config, factory, approval modes, hover/land timing, target regression, View controls and MAVLink status; focused App/View tests and SITL smoke test if feasible.

Scope/defaults: Hover/land 20 m above launch and 10 s continuous hover. Existing target LOS behavior remains except preflight Visual gate. No new target guidance tuning.

Verification outcome: App tests passed (14), the wheel includes the hover YAML, and a headless Betaflight SITL flight reached about 20 m, hovered 10 seconds, landed, confirmed disarm, and exited 0 without View. Task JSON parsed and `git diff --check` passed.
