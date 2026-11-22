# Colored mission state logs

Goal: Show every accepted mission phase transition in the diagnostic log with a colored destination state on color-capable terminals.

Implementation steps: Log once in the shared state-machine callback, so hover/land and target-hit use the same path. Use distinct colors for fault, landing/disarming, disarmed, and active phases. Preserve plain text in the file sink. Add focused tests for accepted, no-op, rejected, and target-only transitions; update operator documentation.

Interface changes: No CLI or protocol changes. Adds an INFO diagnostic line for each actual phase transition.

Verification: The locked App suite passed (79 tests). The focused test confirmed green, red, yellow, and cyan ANSI output, plain file text, one line for each accepted transition, and none for no-op or rejected requests. `git diff --check` and local documentation links passed.

Scope/defaults: No log for unchanged or rejected phase requests. Keep existing logger configuration and tracking logs.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
