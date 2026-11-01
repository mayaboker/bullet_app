# Runtime parameter management

## Goal
Implement Bullet View parameter browsing/search/categories/favorites, per-row MAVLink edits, complete mission YAML export, selected live LOS gains, and remote runtime logs/recorded events.

## Implementation
Shared typed parameter catalog and MAVLink 2 extended parameter protocol; bounded worker/control queues; owner-thread validation/application and post-application ACK; state-preserving LOS/hold tuning; stopped-only structural edits; version/revision checked downloads; 20 Hz streaming and bounded retries; Qt MVP parameter table and local favorites.

## Interfaces
PARAM_EXT_REQUEST_LIST/READ, PARAM_EXT_VALUE/SET/ACK. Shared catalog IDs and schema analysis_parameters.json; analysis/app/parameters recorded events. Public apply_live_tuning methods preserve navigation/filter/controller state. YAML uses existing degree keys. No remote file writes.

## Verification
Catalog/codec/export round trips; runtime ordering/rejection/idempotence/state preservation; loss/retry/stale/source handling; Qt offscreen UI; UDP integration and desktop screenshot. Run affected package suites. Results: 476 affected-package tests passed; the expanded 12-test codec suite
also passed after preserving robust parsing in the per-connection MAVLink 2
adapter. Real UDP tests covered complete synchronization, live edits and rejection;
MCAP readback verified parameter events and effective configuration. Offscreen Qt
covered filters, favorites, edits and export; an actual X11 desktop preview was
rendered and inspected. All four affected packages built as source distributions
and wheels. No active flight configuration was changed; no commits or pushes.

## Scope and defaults
Live yaw, azimuth and elevation gains only; COMMIT rejects edits; other changes require stopped mission. No firmware bridge, import, automatic reconnect, remote persistence, commits or pushes.

## Status
Implemented; verification passed

## Implementation commit
`cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d`

## Repository snapshot record
Recorded snapshot: `cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d` (bullet_app repository). This is the new local
baseline containing the current implementation, not a recovered historical commit.
