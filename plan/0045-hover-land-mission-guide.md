# Hover-land mission guide

Goal: Document the implemented `hover-land` mission in enough detail to explain startup, data flow, approval modes, phase transitions, altitude control, landing, safety behavior, and operator use.

Implementation steps: Read the CLI, mission factory and profile, shared mission state machine, altitude controller, App control loop, View link, and workspace launcher. Add `docs/hover-land-mission.md` with a phase diagram, timing and threshold table, run instructions, and source links. Link it from the App README.

Interface changes: Documentation only; no CLI, configuration schema, MAVLink/MSP protocol, or runtime behavior changes.

Verification: Compared the phase and control descriptions with the CLI, mission factory, `MissionBase`, `AltitudeControl`, control loop, and View link. Checked that all local Markdown links resolve and both Mermaid fences are present. `git diff --check` passed. No runtime tests were run for documentation-only changes.

Scope/defaults: Describe current SITL behavior, including the packaged 20 m and 10 s profile and both manual and auto approval modes. Distinguish Betaflight feedback from App-to-View status.

Status: Implemented; documentation verified. Implementation commit: Pending.
