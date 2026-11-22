# Mission blackboard and step context

Goal: Give Bullet App one process-global blackboard for current MAVLink, validated visual, and derived target data, and pass it to both missions through a timestamped `StepContext`.

Implementation steps: Add a singleton `Blackboard` with explicit run reset and typed MAVLink, visual, and feature sections. Add immutable `StepContext` with monotonic timestamp and derived vehicle accessors. Populate the board once per App tick before mission handling; move mission, View, and tracking consumers to that shared data. Change both mission `step` methods to accept only the context. Update documentation and tests.

Interface changes: `MissionBase.step(context)` and `TargetMission.step(context)` replace the positional tick arguments. The blackboard owns sensor observations and target-derived outputs; mission phase, timers, controller, and transition conditions stay in mission classes. No MAVLink/MSP wire changes.

Verification: The full App suite passed with 45 tests, including new singleton reset, MAVLink timestamp, missing-altitude, visual-copy, and target-output checks. Existing flight, safety, View, and tracking tests pass with the new context interface. `uv lock --check` and `git diff --check` passed. No live SITL run was performed.

Scope/defaults: One process-global board reset at each `run_mission` entry and shared across manual flights. Use existing `TelemetrySnapshot` and current target-derived outputs; do not add a generic feature store or move sensor conditions into state-machine guards.

Status: Implemented; tests and locked dependency check passed. Implementation commit: Pending.
