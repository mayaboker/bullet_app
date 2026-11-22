# Separate tracker and simulation inputs

Goal: Keep real tracker observations independent from simulator-only target contact.

Implementation steps: Replace `ingress.py` with `tracker.py` and `simulation.py`, each owning its own subscriber. Add YAML simulation settings and a SITL config. Open simulation contact only for target-hit when enabled. Move contact state to the blackboard's simulation data. Update launch paths, tests, and docs.

Interface changes: Add `simulation.enabled` and `simulation.contact_endpoint` YAML fields. Keep `--contact-endpoint` as an override that requires simulation enabled. Rename target contact observation to `observe_simulation_contact`.

Verification: The locked App suite passed (77 tests), including config validation, subscriber parsing and cleanup, disabled and enabled contact paths, confirmed hit, and the three-second Commit timeout. The wheel contains both YAML configs and subscriber modules. `git diff --check`, workspace diff check, shell syntax check, CLI help, and local documentation links passed.

Scope/defaults: Bundled config disables simulation. SITL config enables it. Without contact, target-hit retains its three-second Commit timeout. Preserve existing uncommitted work.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
