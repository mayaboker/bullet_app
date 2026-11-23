# Main-owned manual restart retry

Status: Implemented. Implementation commit: Pending (no commit requested).

## Goal
Let a View-requested manual TCP SITL restart unwind one flight session and retry `run_mission()` from `main()`, including automatic FCU connection retries.

## Implementation
- Validate restart as before, send `IN_PROGRESS`, then raise a dedicated restart signal through the session cleanup.
- Keep an App epoch across `run_mission()` calls. Reopen the View UDP link and flight resources on each attempt; advance the epoch only after fresh disarmed FCU telemetry.
- Retry FCU connection failures with a one-second delay after an accepted restart. Leave initial startup, automatic mode, serial connections, and non-connection setup failures on their existing safety paths.
- Let View complete from a new ready epoch and fresh disarmed readback, without requiring a final ACK.

## Interface
MAVLink command 31013 and `APP_STATE`/`APP_EPOCH` remain unchanged. An accepted restart sends an `IN_PROGRESS` ACK before the old View socket closes.

## Verification
118 App tests passed, including focused exception handoff, retry, and cleanup tests. View tests cover epoch and readiness completion. No live SITL run was performed.

## Scope and defaults
Manual TCP SITL only. Restart permission remains false by default. FCU retries continue until success or process stop.
