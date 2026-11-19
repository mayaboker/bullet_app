# Manual target mission settings

Goal: let Bullet View override target-hit takeoff height and pre-search hover duration in manual mode.

Implementation: accept an atomic two-value settings command while disarmed or armed idle, validate both values, publish the applied hover duration with height, and retain the startup defaults until changed.

Interfaces: MAVLink command 31012 uses param1=height_m and param2=hover_s. NAMED_VALUE_FLOAT HOVER_S reports the applied duration. Only manual target-hit accepts the command.

Verification: focused mission and View-link command tests, then the App suite.

Scope/defaults: height 0.1–100 m; hover 0.1–600 s; target defaults 10 m and 10 s. Settings lock once flight starts. No commit requested.

Status: Implemented; 32 App tests passed. Implementation commit: Pending (no commit requested).
