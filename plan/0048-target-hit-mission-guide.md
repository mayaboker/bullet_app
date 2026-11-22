# Target-hit mission guide

Goal: Explain the current target-hit mission from setup and sensor ingress through flight phases, LOS guidance, contact confirmation, and failure handling.

Implementation steps: Review the target mission, shared flight state, App loop, target ingress, estimator, navigator profile, View link, and launcher. Add a source-linked `docs/target-hit-mission.md` with a phase diagram and operator guidance. Link it from the App README.

Interface changes: Documentation only; no CLI, profile schema, wire protocol, or flight behavior changes.

Verification: Compared phase transitions, timing thresholds, input validation, and failure paths with the current App source. All 15 local links resolve, both Mermaid fences are present, and `git diff --check` passed. No runtime tests were run for this documentation-only change.

Scope/defaults: Document the current red-truck SITL path and both approval modes. Distinguish detector boxes, MAVLink feedback, and simulator contact; do not describe future recording work as implemented.

Status: Implemented; documentation verified. Implementation commit: `fae543009b6ec8b0984131bde59e5ecf0ea1ff99`.
