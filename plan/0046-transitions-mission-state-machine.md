# Transitions mission state machine

Goal: Make the shared and target-hit mission phase logic easier to read by using `transitions.Machine` with ordered `resolve()` transitions, while preserving flight behavior.

Implementation steps: Add and lock the `transitions` dependency. Define the phase graph, guards, and entry actions in the mission state machine. Split shared `step()` into telemetry validation, measurement update, transition resolution, and command construction. Route target-hit phase changes through the same graph without changing guidance calculations. Add focused transition and safety checks, and update the hover-land guide to explain phase ownership and controller reset behavior.

Interface changes: Keep `MissionBase.request`, `step`, `fault`, `Phase`, readiness, and View-facing behavior stable. The machine becomes the owner of runtime phase changes; no MAVLink or MSP wire changes.

Verification: The pre-refactor focused baseline was 21 passing mission tests. After implementation, the full App suite passed with 42 tests, including new graph, entry action, and safety timing checks. `uv lock --check` and `git diff --check` passed. No live SITL run was performed.

Scope/defaults: Refactor both hover-land and target-hit. Preserve thresholds, controller gains, approval modes, synchronous request handling, and error messages. Use an ordered `resolve()` trigger with explicit transition priority.

Status: Implemented; tests and locked dependency check passed. Implementation commit: Pending.
