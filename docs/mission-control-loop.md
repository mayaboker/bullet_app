# Bullet App mission control loop

Bullet App is the controller between Betaflight and the operator. It reads Betaflight telemetry over MAVLink, chooses an RC command in a paced 50 Hz loop, and sends that command over MSP. Each Betaflight link can use TCP or serial as selected in the [connection YAML](../src/bullet_app/app.yaml); the diagram shows the default TCP ports. Bullet View provides operator requests and receives App state on a separate MAVLink link. The `target-hit` mission consumes tracker boxes over ZeroMQ. The optional [SITL config](../src/bullet_app/app.sitl.yaml) also enables simulator contact. App does not read simulator pose or other ground-truth flight feedback.

## Component relationships

```mermaid
flowchart LR
    FC["Betaflight SITL / flight controller"]
    View["Bullet View"]
    Visual["Bullet Visual detections"]
    Contact["Simulation contact publisher"]
    Bagger["mcap_bagger"]

    subgraph App["Bullet App"]
        Entry["main.py + CLI<br/>logging and options"]
        Runner["app.py<br/>run_mission, 50 Hz"]
        Telemetry["beta_client<br/>MAVLink receiver thread"]
        Blackboard["Process-wide blackboard<br/>MAVLink, visual, simulation, target features"]
        Context["StepContext<br/>blackboard + monotonic time"]
        Msp["beta_client<br/>MSP TCP transport"]
        Link["ViewLink<br/>commands, ACKs, state"]
        Tracker["TrackerSubscriber<br/>ZMQ boxes"]
        Simulation["SimulationContactSubscriber<br/>optional ZMQ contact"]
        Hover["HoverLandMission"]
        Target["TargetMission"]
        Base["MissionBase<br/>shared flight phases"]
        Altitude["AltitudeControl<br/>throttle from fresh altitude"]
        Estimator["TargetSensorEstimator<br/>aligns target and sensors"]
        Navigator["LosNavigator<br/>LOS guidance"]
        Diagnostics["RuntimeDiagnostics<br/>rate, flight, and LOS logs"]
        Tracking["TrackingLog<br/>target guidance checkpoints"]
        Publisher["Optional background publisher<br/>MsgPack + ZMQ"]
    end

    Entry --> Runner
    FC -->|"MAVLink TCP 5762"| Telemetry
    Telemetry --> Runner --> Blackboard
    Runner --> Context --> Hover
    Context --> Target
    Blackboard --> Context
    Runner -->|"RC channels"| Msp -->|"MSP TCP 5761"| FC
    View <-->|"MAVLink UDP 14550"| Link
    Link <-->|"operator requests / App state"| Runner
    Visual -->|"bbox, target only"| Tracker --> Runner
    Contact -->|"contact, when enabled"| Simulation --> Runner
    Hover --> Base
    Target --> Base
    Base --> Altitude
    Target --> Estimator --> Navigator --> Target
    Target -->|"accepted bbox, navigation output"| Blackboard
    Blackboard --> Link
    Blackboard --> Diagnostics
    Blackboard --> Tracking
    Blackboard --> Publisher -->|"timestamped opaque payloads"| Bagger
    Runner --> Diagnostics
    Target --> Tracking
```

`run_mission` creates one selected mission. Both implementations share `MissionBase` for arming, takeoff, altitude hold, landing, and disarming. `TargetMission` adds search, centering, settling, line-of-sight (LOS) guidance, and contact handling. The target estimator aligns timestamped camera observations with MAVLink attitude and altitude before passing them to `LosNavigator`.

`run_mission` resets the process-global `Blackboard` before creating a mission. Each tick replaces its immutable MAVLink snapshot, adds validated target observations, polls View commands, and passes `StepContext(blackboard, now_s)` to `mission.step()`. View reads current telemetry from the board; tracking diagnostics read its accepted bbox, optional simulation contact, and current target navigation input/result. Contact lives in `blackboard.simulation`, separate from `blackboard.visual`. An optional [background publisher](telemetry-publishing.md) reads the board on its own schedule and sends MsgPack values to `mcap_bagger`; the mission tick does not encode or send them. Source receive and capture timestamps remain attached to their data. Mission phase, timers, and the altitude controller remain on the mission; the `transitions` machine still checks legal phase changes rather than reading sensor conditions directly from the blackboard.

At startup, App also queries `MSP_BOARD_INFO` on the MSP connection and reports whether it identified a SITL firmware target in its View heartbeat. This check does not measure whether the simulator is exchanging data.

## One control tick

```mermaid
flowchart TD
    A["Read MAVLink snapshot into blackboard"] --> B["For target-hit: observe TIMESYNC and poll tracker;<br/>poll contact only when simulation.enabled"]
    B --> C["Poll View heartbeat and commands"]
    C --> D["Check firmware arm state<br/>and telemetry freshness"]
    D --> E{"Fresh heartbeat and altitude,<br/>no mission fault?"}
    E -->|Yes| F["Step selected mission with StepContext"]
    F --> G["In auto mode, request Arm or Start<br/>when its phase permits"]
    E -->|No| H["Fault if freshness timeout elapsed;<br/>choose throttle 1000, disarmed"]
    G --> I["Send RC command over MSP"]
    H --> I
    I --> J["Publish state and fresh telemetry<br/>to View; update diagnostics"]
    J --> K["Wait for next 20 ms tick"]
    K --> A
```

The MAVLink receiver runs in its own thread and updates a thread-safe snapshot. App requests `VFR_HUD` and `ATTITUDE` at 50 Hz; `target-hit` and source-timestamped telemetry publication also enable `TIMESYNC`. The control loop reads the latest values without waiting for a new packet. It uses `HEARTBEAT.base_mode` to observe whether Betaflight is armed and `VFR_HUD.alt` as its altitude measurement. The shared altitude controller updates its throttle estimate only when a new altitude sample arrives.

In manual mode, View's Arm, Start, Land, and Disarm commands enter through `ViewLink.poll`; manual target missions also accept height changes while idle. View heartbeat loss makes an armed idle mission disarm or an airborne mission land. In auto mode, App requests Arm and Start itself after the shared mission reaches the appropriate phase. A target mission additionally requires fresh TIMESYNC before Arm or Start.

The loop faults if the vehicle heartbeat is older than 3 seconds or altitude remains unavailable after the 10-second startup allowance or later becomes older than 1 second. It then sends a disarmed, minimum-throttle command. `ViewLink.publish` sends App heartbeat, state, fresh attitude and relative altitude, plus status updates at their own rates; these are App-originated MAVLink messages, separate from the incoming Betaflight stream. The shared state-machine callback logs every accepted phase transition once, with a colored destination on color-capable terminals and plain text in the log file. `RuntimeDiagnostics` owns rate monitoring and periodic flight and LOS logs. `TargetMission` owns `TrackingLog` and observes the sent RC command through the shared post-send hook. Neither logger controls flight.

## Mission paths and shutdown

- **Hover/land:** prearm settle → disarmed → arming → armed → takeoff → hover → land → disarming → disarmed. Hover completion starts landing.
- **Target-hit:** the same initial phases lead from hover to search → center → settle → LOS → commit. A confirmed contact disarms with a hit result; search or guidance failures lead to landing. Its estimator and navigator are active for the target path.
- **Shutdown:** auto mode returns after the completed attempt or fault once disarmed; manual mode remains available after a normal landing. When the loop exits, its `finally` block attempts three disarmed minimum-throttle MSP commands before closing resources.

The diagram follows [the loop](../src/bullet_app/app.py), [the shared mission state machine](../src/bullet_app/missions/base.py), [the target mission](../src/bullet_app/missions/target_hit.py), [the View link](../src/bullet_app/view_link.py), [tracker input](../src/bullet_app/tracker.py), and [optional simulation contact](../src/bullet_app/simulation.py).
