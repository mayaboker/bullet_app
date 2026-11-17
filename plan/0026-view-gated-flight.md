# View-gated Auto and Manual flight

Status: Implemented; focused tests and Auto/Manual SITL flights passed. Implementation commit: Pending (no commit requested).

Goal: App starts disarmed, waits for Bullet View Arm and Start, runs Auto 10 m/10 s/land or Manual selected-height hold until Land, and remains available for repeated flights.

Implementation: Add app-owned MAVLink control and telemetry sockets on loopback UDP 14551/14550; extend the flight state machine for operator commands, GCS heartbeat loss, and variable-height timeouts. Keep beta-client as the SITL adapter. Advertise observed arming, readiness, applied height, mode and phase. Send addressed MAVLink 2 ACKs for Arm, Start, Height, Land and Disarm.

Interface: `bullet-app --mode auto|manual` defaults to Auto. Manual height is 0.1–100 m in 0.1 m steps. View is the only operator peer; app owns SITL/renderer independence.

Verification: Five focused app tests passed. A separate Betaflight SITL and App completed Auto Arm/Start/10 m hover/land and Manual height 3 m/Arm/Takeoff/hold/Land, with View's MAVLink flight client receiving addressed ACKs and observed state. A 0.1 m Manual trial also took off and landed after the near-ground climb adjustment, though its hover oscillated. The offscreen View process sent a heartbeat seen by App. No commit requested.
