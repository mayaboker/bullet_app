# Simulation time and camera frame in status logs

## Goal
Replace visible catalog-number prefixes in Bullet View logs with simulation seconds from scenario start and the associated camera image sequence.

## Implementation steps
- Add a shared optional time/frame STATUSTEXT prefix and parser; repeat context on each UTF-8 chunk within the 50-byte limit.
- Publish status simulation time and processed image sequence; use a latest-image fallback only from the same physics/episode.
- Extend immutable viewer status models with optional metadata; retain raw text and message code for descriptions/details.
- Display [27.075 s | frame 4809]; unavailable fields use an em dash. Preserve clear-log, colors and checkbox-controlled scrolling.

## Interfaces
Optional simulation_time_s and image_sequence formatter/model fields. Legacy indexed and unindexed status messages remain supported. No new MAVLink messages or FlightLens numbering changes.

## Verification
Shared parsing/chunking and UTF-8 limits, remote image fallback and scenario resets, viewer rows/details and existing controls. Results: all 361 tests in bullet_view, bullet_msgs and bullet_app passed.
Desktop X11 preview inspected with simulation time/frame rows, message code and
raw text retained in details. All three affected packages built as source
distributions and wheels. No running mission was changed; no commit or push.

## Scope/defaults
Simulation time, not viewer wall time. Camera sequence, not control-tick or replay ordinal. No guessed metadata. Restart updated app/viewer once.

## Status
Implemented; verification passed

## Implementation commit
`cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d`

## Repository snapshot record
Recorded snapshot: `cb6e3bf6e3ac7dbebd38e9ec28cd5575fce8759d` (bullet_app repository). This is the new local
baseline containing the current implementation, not a recovered historical commit.
