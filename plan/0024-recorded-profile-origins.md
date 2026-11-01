# Recorded file origins and application-only parameters

## Goal
Record source configuration/profile paths in future MCAP snapshots and restrict FlightLens Recording parameter display/search to the actual application/LOS profile.

## Implementation
- Add optional files metadata to the shared analysis_config schema. Record mission_profile and transport_config in app snapshots, including snapshots after runtime edits/reloads.
- Record physics_config, render_config and transport_config origins in physics snapshots; retain caller ownership and optional API compatibility.
- Retain file metadata with immutable recorded snapshots and show names with full-path tooltips. Never infer old filenames or open today's files in the viewer.
- Remove the physics parameter selector/table mode. Display/search application profile parameters only; keep runtime application change events visible.

## Interfaces
Optional files mapping on analysis_config packets and ConfigurationSnapshot. MissionRuntime accepts an optional transport_config path; physics set_analysis_config accepts optional files. No editable file-name tuning fields or MAVLink changes.

## Verification
Producer/schema tests for file origins and parameter-update snapshots, compatibility without metadata, reader round trips, application-only search and old recording labels; Verification results: FlightLens desktop suite passed (91 tests); bullet_app and
bullet_msgs suites passed (339 tests); physics publisher TCP and external-runner
configuration tests passed (5 tests). Focused provenance/runtime/view checks also
passed offscreen (18 tests, included in those suite totals). All four affected
packages built as source distributions and wheels. A synthetic MCAP created from
producer snapshot helpers was opened on the desktop: all three source filenames
were visible and search was limited to application parameters. Screenshot:
/tmp/flight-lens-application-origins.png. No live scenario/profile values were
changed; old recordings were not modified or assigned guessed filenames.

## Defaults and scope
Future recordings only; old MCAPs remain Not recorded. File names are provenance, not tunable controller settings. Effective parameters remain authoritative after runtime edits. No commits/pushes requested.

## Status
Implemented; verification passed (not committed)

## Implementation commit
Pending
