# MAVLink recording control

## Goal and implementation
Relay USER_2 start/stop to a dedicated recorder lifecycle in both detector and
mission modes; confirm through asynchronous ACK/status. Select mavlink or godot
authority; preserve reload, parameters and flight state. See workspace
plan/0005-production-bringup.md for the full contract and verification scope.

## Interfaces
Optional recording CLI options; default disabled for existing application runs.
USER_2=31011, param1 1=start/0=stop. Dedicated ZMQ state/status, no simulation state.

## Verification
MAVLink dispatch, real recorder roundtrip, idempotency, wrong target, timeout,
Godot rejection and existing app regressions.

## Status
Implemented; verification passed. Not committed.

## Implementation commit
Pending

## Verification results
- 20 bringup tests passed: configuration/dry-run, process startup/failure/signals,
  readiness timeout, partial startup and log-thread failure cleanup, real local
  MAVLink/ZMQ/MCAP roundtrip, Godot isolation and pending-command shutdown ACK.
- 173 application tests, 92 recorder tests, 182 shared-message tests and 8
  workspace tests passed (475 total). Existing simulation recording, reload and
  parameter handling remain covered by their regression suites.
- Tested new recording error paths: bad/stale/reordered messages, confirmation
  timeout, disk-open failure, writer failure, silence timeout, duplicate commands,
  closed-session protection and a fresh Start after recorder closure.
- Built /tmp/bullet-bringup-bundle-final with eight local runtime wheels plus
  locked third-party wheels (113 MB, Linux x86_64 / Python 3.12 build host).
  Installed offline into /tmp/bullet-bringup-installed and ran from /tmp with
  PYTHONPATH and display variables unset. Installed MAVLink start/stop/readback
  and pending-command shutdown passed. All eight local packages resolve from
  that venv; no simulator, Qt viewer or pytest packages are installed there.
- MAVLink/Godot dry-runs validated without creating logs or recordings. Existing
  dependency versions are preserved; uv lock --check and changed-file whitespace
  checks passed. No real camera capture or flight actuation was performed.
- Applied exception-risk-review to thread/socket/process cleanup. Fixed partial
  log-thread startup cleanup, worker-error finalization reporting, pending ACKs
  on app shutdown and restarting a previously closed recording session.
- New bringup repository is initialized on main; no commits, remotes, pushes or
  systemd services were added. Process logs/build wheels are outside source or
  ignored. Test-owned child processes were terminated.

Verification logs: /tmp/bringup-*-final.log; installed check:
/tmp/bringup-installed-final.log. Local sources and wheel payloads were compared.
