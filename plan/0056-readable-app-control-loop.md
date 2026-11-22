# Readable App control loop

Goal: Make `run_mission` a readable sequence of setup, input collection, safety-controlled stepping, output, diagnostics, pacing, and shutdown without changing flight behavior.

Implementation steps: Extract SITL board identification, input collection, and telemetry-gated mission stepping into focused helpers. Move persistent rate/phase/result/TIMESYNC/LOS diagnostics into a dedicated component; retain command sending, View publication, auto completion, pacing, resource ownership, and final disarm visibly in `run_mission`. Preserve the existing ordering of all side effects.

Interface changes: Keep `run_mission(options) -> int`, the CLI, MAVLink/MSP values, logs, and shutdown behavior unchanged. Add only private helper APIs and an internal diagnostics component.

Verification: The locked App suite passed (57 tests). Focused tests cover stale feedback, the altitude startup allowance, command/publication/diagnostic ordering, automatic completion, and three final disarm sends. `git diff --check` and the control-loop documentation links passed. The root README was reviewed and remains accurate.

Scope/defaults: Readability refactor only. Preserve the current uncommitted `AGENTS.md` and plan 0055 edits.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
