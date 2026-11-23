# Match Angle mode RC mapping to Betaflight ACTUAL rates

## Goal

Make target-hit roll and pitch demands produce their requested Angle mode attitude in the pinned Betaflight SITL profile. A 10° pitch demand currently sends 1611 PWM and settles near 3°.

## Implementation

- Replace the linear roll/pitch-to-PWM conversion with the inverse of Betaflight's ACTUAL rate curve and Angle mode normalization.
- Use the pinned SITL settings: 45° angle limit, 70°/s center sensitivity, 670°/s maximum rate, and zero expo. Keep the conversion bounded to the RC range.
- Add focused checks for zero, signed 10°, and limit demands. Explain the profile dependency in the root README.

## Interface changes

No public API or YAML fields. The RC output of target-hit LOS roll and pitch demands changes.

## Verification

- Target mission tests: 19 passed. Complete Bullet App suite: 111 passed.
- SITL target-hit attempt: RC pitch reached 1722 PWM and measured tilt about 10° after the slew ramp. The mission reported `LOS recovery_timeout` after a LOS deviation near LOS+7 s and landed without contact. The 10° trajectory needs separate guidance tuning.

## Scope and defaults

Match the repository's pinned Betaflight SITL profile. Other Betaflight rate profiles require matching App conversion settings before flight.

## Status

Implemented; 111 tests passed, SITL angle response confirmed, and the faster trajectory did not hit. Implementation commit: Pending (no commit requested).
