# Local readiness enum for the App–View MAVLink link

Goal: Give App readiness codes meaningful names without adding a shared Python protocol dependency between Bullet App and Bullet View.

Implementation steps: Define an App-local `IntEnum` for the existing readiness values 0–4; return its members from both missions; use the named flying value for heartbeat status. Preserve the numeric `MIS_READY` MAVLink field.

Interface changes: `mission.readiness` becomes an `IntEnum` that remains comparable to integers and converts to the same float on the wire. No MAVLink message, field, or value changes.

Verification: The App suite passed (52 tests), including readiness value and type checks and existing target TIMESYNC and heartbeat tests. The View suite passed (90 tests, one skipped). Both repositories passed `git diff --check`.

Scope/defaults: Keep the enum in Bullet App. Bullet View defines its own local interpretation of the numeric MAVLink contract; no `bullet_msgs` flight-control helper is shared between the two programs. Preserve existing uncommitted App changes.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
