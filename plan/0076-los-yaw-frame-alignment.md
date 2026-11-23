# Align LOS yaw with the camera frame

## Goal

Correct the yaw sign used by LOS bearing estimation and heading hold. In the 90° spawn run, a positive MAVLink yaw sweep moves the fixed truck from left toward image center, while the LOS attitude conversion interprets the same sweep in the opposite direction.

## Implementation steps

1. Convert the observed MAVLink yaw to the camera/world positive yaw convention without negating it. Keep the existing roll and pitch conversions.
2. Count the search sweep with the same yaw sign.
3. Add regression tests for image-bearing invariance during yaw and search-sweep accumulation.
4. Compare the failed run's ground-truth lateral position with its log, then run a fresh target-hit simulation and inspect its tracking log.

## Interface changes

None. The mission's internal attitude conversion changes.

## Verification

`uv run pytest -q`: 121 passed. `git diff --check` passed. In a live 90° spawn run on 2026-11-23 at 15:52, App entered LOS at 15:53:27, kept the target near image center through most of the descent, and reported confirmed simulator contact (`hit`) at 15:53:39. The preceding run had drifted sideways and failed with `LOS recovery_timeout`.

## Scope and defaults

Change yaw interpretation in the target-hit mission only. Preserve the slower settle gate and LOS gains.

## Status

Implemented. Implementation commit: Pending (no commit requested).
