# Merge Alice attitude timing into Bob

## Goal

Accept fresh Isaac SITL attitude samples when TIMESYNC maps them slightly into the future, while retaining Bob's boot-time alignment for Bullet LOS tracking.

## Implementation steps

1. Merge `origin/alice` into `bob`.
2. Use vehicle boot time mapped through TIMESYNC when valid. For a bounded future offset, use the fresh local receipt time.
3. Test both timestamp paths and rejection of a large future offset.

## Interface changes

None.

## Verification

`uv run --frozen pytest -q`: 123 passed. The tests cover boot-time mapping, the bounded future-time fallback, and rejection of a larger future offset.

## Scope and defaults

The receipt-time fallback applies only when the mapped time is future-dated by no more than the configured maximum attitude age. Other invalid mappings remain rejected.

## Status

Implemented. Implementation commit: `1e3089b`.
