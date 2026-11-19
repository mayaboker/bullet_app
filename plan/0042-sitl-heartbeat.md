# SITL identification in App heartbeat

Goal: Query the connected Betaflight board over MSP during App startup and report whether its target is SITL to Bullet View.

Implementation: Add a bounded, validated MSP_BOARD_INFO request in beta_client; make App query it once before flight control and set bit 0 of its heartbeat custom_mode only for a SITL reply. Continue with a clear bit and diagnostic on any other or missing reply. Document the wire bit and startup behavior.

Interface: App heartbeat custom_mode bit 0 means the MSP board identifier was SITL at App startup. A clear bit means SITL was not confirmed; it does not identify hardware or simulation connectivity.

Verification: beta_client's 34 unittest cases and App's 36 pytest cases passed. A live Betaflight SITL answered `MSP_BOARD_INFO` with `SITL`, and a running App sent `HEARTBEAT.custom_mode = 1` to a UDP listener on View's port.

Scope/defaults: No flight gate or firmware changes. Status: Implemented and verified. Implementation commit: Pending.
