# Mission action enum

Goal: Replace string action identifiers inside Bullet App mission requests with an App-owned enum.

Implementation steps: Define `Action` for arm, start, land, disarm, emergency, and height. Require it in `MissionBase.request` and `TargetMission.request`; update automatic Arm/Start and MAVLink command decoding to pass enum members. Use enum values only for diagnostic text and keep the current ACK action IDs, denial behavior, and separate `request_height` path.

Interface changes: `mission.request(action, ...)` accepts `Action` instead of a string and rejects non-enum actions with `ValueError`. `_command()` returns an `Action` and optional value. No MAVLink command IDs, parameters, or result fields change.

Verification: The locked App suite passed (53 tests). Tests cover enum decoding, rejection of raw strings, manual/automatic gates, target TIMESYNC and one-attempt gates, and unchanged View ACK action IDs. `git diff --check` passed.

Scope/defaults: Keep `Action` local to Bullet App; Bullet View continues using MAVLink IDs and its own local action strings. Preserve the existing unrelated blank-line change in `base.py`.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
