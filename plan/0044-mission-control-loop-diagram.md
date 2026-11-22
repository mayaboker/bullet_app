# Mission control loop diagram

Goal: Explain Bullet App's current components and 50 Hz mission control loop in a Markdown document under `docs/`.

Implementation steps: Review the entry point, telemetry and command adapters, mission implementations, target-only inputs, diagnostics, and shutdown path. Add a Mermaid block diagram for component relationships and a compact loop diagram for execution order. Explain the shared flight phases, target-only guidance path, freshness checks, and command flow. Link the document from the root README.

Interface changes: Documentation only; no CLI, wire protocol, or application behavior changes.

Verification: Compared both diagrams and the phase, timing, and failure descriptions with the current source. Confirmed both Mermaid fences and all five source links are present and valid; a local Mermaid renderer is unavailable. `git diff --check` passed. No runtime tests were run for documentation-only changes.

Scope/defaults: Describe the current hover-land and target-hit missions, the separate MAVLink/MSP/View links, and direct simulation contact input without suggesting App receives simulation ground truth. Leave plan 0043 and application code unchanged.

Status: Implemented; documentation verified. Implementation commit: Pending.
