# Heavy-quad descent limit of 5.5 m/s

## Goal
Allow a maximum commanded descent of 5.5 m/s in both heavy-quad mission profiles.

## Implementation steps
- Change tuning.max_descent_m_s from 3.5 to 5.5 in workspace config/bullet_profile_heavy_quad.yaml and config/betaflight_los_heavy_quad.yaml.
- Preserve all other profile values and controller behavior.
- Provide launcher commands that explicitly select the matching heavy-quad physics and mission profiles.

## Interfaces and defaults
No API, schema or MAVLink changes. Positive max_descent_m_s=5.5 permits a vertical-speed reference down to -5.5 m/s, subject to slew and controller supervision. This does not guarantee measured speed tracking or fix roll lag. Existing running sessions retain loaded settings until restart or a stopped profile reload. Historical recordings remain unchanged.

## Verification
Load both profiles through NavigatorConfig.from_yaml and verify effective max_descent_m_s=5.5. Compare pre/post YAML values and effective configuration to ensure no unrelated tuning changes. Future startup output/new MCAP application configuration should confirm the active limit; no live restart or flight is part of this profile edit.

Results: both profiles loaded successfully with effective max_descent_m_s=5.5.
Exact text comparisons confirmed only the requested value changed in each file.
Parsed YAML and normalized effective configuration comparisons also confirmed
that all other values were preserved. No new tests were added for this value-only
edit, and no running process or historical recording was modified.

## Launch and active-setting verification
Run one of these commands from the workspace root. Absolute profile arguments
are intentional because the launcher runs workers in different directories.

Internal controller:
```bash
uv run --locked --all-packages python bullet_render/tools/run_flight.py --physics-config "$PWD/config/internal_heavy_quad.yaml" --mission-config "$PWD/config/bullet_profile_heavy_quad.yaml"
```

Betaflight:
```bash
uv run --locked --all-packages python bullet_render/tools/run_flight.py --physics-config "$PWD/config/betaflight_heavy_quad.yaml" --mission-config "$PWD/config/betaflight_los_heavy_quad.yaml"
```

Restart the existing launcher before using these commands. In the new application
startup profile or MCAP analysis/app/config settings, confirm
tuning.max_descent_m_s=5.5. FlightLens Recording shows this application parameter.
This change does not modify the launcher's default profile selection.

## Status
Implemented; both profile loads and unchanged-value checks passed. Not committed.

## Implementation commit
Pending
