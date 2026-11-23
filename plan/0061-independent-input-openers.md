# Independent tracker and simulation openers

Goal: Give tracker and simulation contact separate setup paths in App.

Implementation steps: Replace `_open_target_inputs` with `_open_tracker` and `_open_simulation`. Call each independently in `run_mission`; poll each independently in `_collect_inputs`. Update focused tests. Preserve concurrent App edits, correcting initialization order and restoring target-hit-only TIMESYNC so the running loop works.

Interface changes: Private App helpers only. No CLI, YAML, or protocol change.

Verification: The locked App suite passed (78 tests), including independent contact polling and both opener paths. `git diff --check` passed.

Scope/defaults: Tracker opens for target-hit. Simulation contact opens only for target-hit when enabled in YAML. No commit requested.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
