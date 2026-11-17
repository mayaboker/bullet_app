# Selectable target mission

Status: Pending
Implementation commit: Pending

Goal: Add a one-attempt red-truck mission selected at App startup while preserving auto and manual flights.

Implementation: Keep View Arm/Start gating; use MAVLink attitude/altitude and timestamped bullet_visual bboxes as the only LOS inputs. Add yaw search, LOS guidance, bounded RC conversion, timeouts, contact-event completion, and mission/result View telemetry. App consumes no simulation pose or feedback.

Interfaces: `--mission target-hit` selects the new mission; the simulation contact topic is the sole simulation input. View receives mission, phase, and result values on the existing MAVLink socket.

Verification: Unit tests for gating, search, guidance, timing and failures; SITL/renderer integration attempt.

Verification outcome: App focused tests passed. Renderer build/export and isolated renderer-to-Visual bbox transport passed. Live launcher check could not start because another Betaflight SITL already owns port 5761.

Follow-up: Fixed the MAVLink mission field name to fit the 10-byte wire limit, published detector readiness, and added denial diagnostics to App logs and View status text. Focused App tests passed.

Scope/defaults: 10 m takeoff, 10 s hover, one measured yaw circle or 60 s, 120 s approach, one attempt per process. Contact disarms; ordinary mission failures land.
