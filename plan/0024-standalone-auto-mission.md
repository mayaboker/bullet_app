# Standalone auto mission

Status: Implemented; focused tests and two-process SITL flight passed. Implementation commit: Pending (no commit requested).

Goal: Run a separate app after the simulation launcher to arm, climb 10 m above the starting altitude, hover for 10 seconds, land, and confirm disarm.

Implementation: Replace the legacy app runtime with a focused auto mission. Use beta-client for MSP RC and MAVLink telemetry, remove old camera/LOS and msp-link dependencies, and keep a standalone uv lockfile. The app owns altitude control, phase timeouts, and cleanup.

Interface: `bullet-app` runs the mission with default local MSP and MAVLink endpoints; the launcher and renderer remain separate processes.

Verification: Four focused app tests passed. A separate launcher with one Betaflight SITL and no renderer completed arm, climb, 10-second hover, landing and confirmed disarm with app exit code 0. No Godot UI changes. No commit requested.
