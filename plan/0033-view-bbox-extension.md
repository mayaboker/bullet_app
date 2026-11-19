# View bbox extension

Status: Implemented; tests and live smoke passed
Implementation commit: `ef32619903337bf9baa67b593b3fe29b51509007`

Goal: Forward validated target detections to Bullet View over the existing MAVLink link.

Implementation: Accept fid-tagged Visual packets and emit a version-1 V2_EXTENSION payload containing only fid and bbox JSON, including no-target results.

Interface: Local bbox packets may include fid; MAVLink V2_EXTENSION message_type 32768 targets View system/component 255/190 on UDP 14550.

Verification: Validation, encoding, and App-to-View tests.

Scope: Target-hit mission only; no second MAVLink port.

Verification outcome: Sixteen App tests passed, including a real pymavlink round trip. Live target stack accepted fid-tagged detections and showed a matching overlay in View.
