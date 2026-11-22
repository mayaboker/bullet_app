# StepContext mission chain

Goal: Use one `StepContext` throughout the shared mission control tick instead of passing separate time, altitude, arm, and View values among its helpers.

Implementation steps: Thread the context from `MissionBase.step` through altitude validation, View-loss handling, phase advancement, altitude control, hover and landing checks, and timeout checks. Change the hover-completion hook in the base and both concrete missions to accept the same context. Derive elapsed time where needed from `context.now_s`.

Interface changes: Step-chain helper methods and `_on_hover_complete` accept `StepContext`. Keep `request`, `fault`, `_enter`, state-machine callbacks, and methods requiring no tick data on their existing interfaces.

Verification: Updated the direct helper call in state-machine tests. The locked App suite passed (54 tests), covering altitude validation, View-loss safety, hover completion, and timeouts. `git diff --check` passed.

Scope/defaults: Preserve control behavior and the existing unrelated uncommitted blank-line edit in `base.py`; remove its trailing spaces for a clean diff.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
