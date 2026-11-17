# Single-port Bullet View link

Status: Implemented and verified. Implementation commit: Pending (no commit requested).

Goal: Send App telemetry first and carry View heartbeat, flight commands, and App ACKs on one bidirectional MAVLink 2 UDP conversation using View port 14550.

Implementation: Replace the separate App telemetry and control sockets with one `udpout:127.0.0.1:14550` socket. Receive View heartbeat and commands on it and return ACKs on it. Keep source/target checks, mission behavior, and SITL connections unchanged.

Interface: Retire App–View UDP 14551. App remains MAVLink system/component 1/1; View remains 255/190.

Verification: All 6 App tests passed. A localhost exchange with View's worker confirmed App-first telemetry, View heartbeat and Arm command, and App ACK through one UDP conversation. Updated the README.
