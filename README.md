# Bullet App

Bullet App is a simulation-only flight controller. Start the `bullet_drone`
launcher first, then start the app and Bullet View as separate processes. The app
uses `beta_client` to read Betaflight MAVLink on TCP 5762 and send MSP RC on TCP
5761. App streams MAVLink to View on loopback UDP 14550; View returns its
heartbeat and flight commands on the same connection, and App replies with
command ACKs. The app does not import the simulator or renderer.

## Run

In the multi-root VS Code workspace, run **Bullet: Run launcher (SITL +
renderer)**, then **bullet_app: Bullet: Run app (manual approval, choose
mission)**, then **bullet_view: Bullet: Run view**. The App task prompts for
`hover-land` or `target-hit`.

From terminals:

```sh
cd /home/user/projects/bullet_drone
uv run --frozen --extra sitl --extra mavlink --extra render launcher

cd /home/user/projects/bullet_app
uv sync --group dev
uv run --locked bullet-app --mission hover-land
# For one unattended flight: uv run --locked bullet-app --mode auto --mission hover-land
# Select another startup profile: --mission-config path/to/hover_land.yaml

cd /home/user/projects/bullet_view
uv run --locked bullet-view
```

The default `hover-land` profile is packaged in
`src/bullet_app/missions/hover_land.yaml`: climb **20 m above launch**, hover
for **10 continuous seconds**, land, and disarm. Pass `--mission-config` to
load different `altitude_m` and `hover_duration_s` values at startup. View
shows the configured height but cannot change it during a run.

`--mode manual` (the default) means View must send **Arm** and **Start hover
mission**; the same mission then flies its configured sequence and remains
available for another flight after landing. `--mode auto` arms and starts
after fresh SITL feedback, needs no View connection, and exits after one
confirmed landing or a fault. View can still issue Land or emergency Disarm
when connected.

In manual approval mode, View sends a heartbeat while connected. If it
disappears for 3 seconds, an armed idle app disarms and an airborne app lands.
Automatic mode does not require View. Stale SITL telemetry causes
an app fault and disarm; restart the app before another flight. Emergency
Disarm in View immediately stops motors and requires an app restart.

`--msp-host`, `--msp-port`, and `--mavlink-endpoint` select nondefault SITL
connections. This controller and its throttle tuning are for SITL only.

## Red-truck target mission

Run **Bullet: Run target scene (SITL + renderer + visual)** from the
`bullet_drone` workspace folder, then **Bullet: Run app (manual approval, choose mission)**,
select **Search for and hit the red truck**, and run
**Bullet: Run view**. The equivalent App command is:

```sh
uv run --locked bullet-app --mode manual --mission target-hit
```

View sends **Arm**, then **Start target mission**. The drone climbs 10 m,
hovers 10 seconds, sweeps in yaw to find and center the red truck, and then
uses `los_tracking` to command roll, pitch, yaw rate and throttle. LOS uses
Betaflight MAVLink `ATTITUDE` and `VFR_HUD` plus the detector's timestamped
bboxes. App does not subscribe to simulation pose or flight feedback. The only
direct simulation input to App is the confirmed red-truck contact event.
Accepted detector results are also forwarded to View as MAVLink `V2_EXTENSION`
messages containing the image `fid` and bbox; View uses that fid to overlay a
matching local renderer image.
Visual results are used during the search after hover; they are not required
before Arm or Start. This target mission retains its existing guidance and
failure behavior pending a separate target-flight plan.
`ATTITUDE.time_boot_ms` is mapped onto the host monotonic clock; `VFR_HUD`
has no source timestamp, so its local MAVLink receive time is used.

Contact disarms and reports **hit**. A full search circle or 60-second search
limit, lost target after LOS recovery, 120-second approach limit, or 3-second
post-commit contact limit triggers landing with a failure reason. This first
version permits one target attempt per App process; restart App and launcher
for another attempt. Existing auto hover/land and manual missions remain
available. The target profile defaults to the sibling `los_tracking` Bullet
profile and can be changed with `--mission-config`.

Run focused tests with `uv run --locked pytest -q`.
