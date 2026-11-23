# Mission mode enum

Goal: Store the selected approval mode as an App-owned enum and use it consistently in mission decisions.

Implementation steps: Define `Mode.AUTO` and `Mode.MANUAL` in the mission base. Convert CLI text at mission construction, update shared and target mission checks, the runtime loop, and View telemetry to compare enum members. Export the enum from the mission and compatibility modules.

Interface changes: `mission.mode` is a `Mode` member. Mission constructors accept a `Mode` member or the existing `auto`/`manual` strings; the CLI continues to expose those strings. MAVLink `APP_MODE` remains 0 for auto and 1 for manual.

Verification: The locked App suite passed (54 tests), including conversion, invalid modes, and the enum-valued mission property. `git diff --check` passed.

Scope/defaults: Preserve the existing default manual mode, existing action-enum work, and all MAVLink wire values.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
