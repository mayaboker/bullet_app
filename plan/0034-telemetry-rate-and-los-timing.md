# Telemetry rate and LOS timing diagnostics

Goal: make upstream ATTITUDE/VFR_HUD rates and image-to-attitude alignment visible while keeping flight behavior unchanged.

Implementation: report measured one-second rates and sample ages; warn after two low-rate windows and announce recovery after two healthy windows. Publish rates to Bullet View. Record LOS image/attitude age, timestamp gap, navigator state, vision reason, and counts of alignment waits and replacements. Raise the Bullet profile image-age limit to 0.2 s.

Interfaces: add `ATT_HZ` and `ALT_HZ` MAVLink named values; add defaulted alignment counters to LOS diagnostics. Existing stale-telemetry flight protection stays in place.

Verification: focused rate-warning, wire, and alignment tests; target-stack diagnostic run.

Scope/defaults: warning below 40 Hz for two one-second windows; recovery at or above 45 Hz for two windows; startup windows ignored. This pass measures alignment and does not change its buffering policy.

Status: Implemented; 17 App tests and 52 focused LOS tests passed. A live target mission reported 46–48 Hz upstream rates and 17 waiting-box replacements in one second before `LOS recovery_timeout`. Implementation commits: `9cfa2de` (App), `069f3b8` (LOS).
