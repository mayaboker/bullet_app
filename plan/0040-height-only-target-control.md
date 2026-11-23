# Height-only target control

Goal: use one manual target-hit height setting for takeoff and hover, with a fixed five-second pre-search hover.

Implementation: accept the height-only MAVLink command before Start; keep one target height in the mission and set its hover duration to five seconds. Retire the two-value settings command.

Interfaces: command 31012 carries height_m in param1 and zero elsewhere; TAKEOFF_H confirms the applied height. HOVER_S continues reporting the fixed duration.

Verification: command decoding, phase guards, takeoff/hover behavior, and the App suite.

Scope/defaults: target-hit manual mode only; valid applied height 0.1–100 m; default height 10 m. Hover-land profile remains unchanged. No commit requested.

Status: Implemented; 32 App tests passed. Implementation commit: `cd22cd8`.
