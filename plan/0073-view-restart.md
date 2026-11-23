# View requested flight-session restart

Status: Implemented. Implementation commit: Pending (no commit requested).

## Goal
Keep Bullet App's View MAVLink link available while a manual SITL flight session is recreated after simulator reset.

## Implementation
- Add an explicit restart permission to App YAML and enable it in the two SITL launch configs.
- Accept project command 31013 only from a fresh, correctly targeted View, in an allowed manual disarmed state.
- Report progress and final command ACK, App state and epoch; reopen flight connections and reconstruct session resources from the original YAML/profile.
- Keep the View link alive on connection failure, and require another explicit restart attempt.

## Interface
MAVLink COMMAND_LONG 31013 with zero parameters; NAMED_VALUE_FLOAT APP_STATE and APP_EPOCH.

## Verification
116 App tests passed. SITL integration was not run during this change.

## Scope and defaults
Manual SITL only. Restart permission defaults to false. No OS process restart.
