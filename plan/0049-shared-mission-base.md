# Shared mission base

Goal: Keep `MissionBase` limited to flight behavior shared by hover-land and target-hit.

Implementation steps: Move target phase edges and airborne phase membership into `TargetMission`. Let shared safety, readiness, land, and RC command handling use the mission's airborne phases. Move height and vision methods to the two concrete missions, and have View read vision status through their common interface.

Interface changes: Preserve `request_height()`, `vision_alive()`, `step(context)`, `Phase`, `Command`, and existing import paths for callers. Remove the target-only height and vision method implementations from `MissionBase`.

Verification: Focused mission and View tests passed (29 tests). The full App suite passed (51 tests), including new checks for landing and View loss from each target phase, readiness, airborne RC output, and concrete mission vision status. `git diff --check` passed.

Scope/defaults: Preserve existing flight behavior and uncommitted work. Keep shared phase definitions and IDs stable for the View protocol.

Status: Implemented and verified. Implementation commit: `fae543009b6ec8b0984131bde59e5ecf0ea1ff99`.
