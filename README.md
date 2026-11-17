# Bullet App

Bullet App is a simulation-only flight controller. Start the `bullet_drone`
launcher first, then start the app and Bullet View as separate processes. The app
uses `beta_client` to read Betaflight MAVLink on TCP 5762 and send MSP RC on TCP
5761. App streams MAVLink to View on loopback UDP 14550; View returns its
heartbeat and flight commands on the same connection, and App replies with
command ACKs. The app does not import the simulator or renderer.

## Run

In the multi-root VS Code workspace, run **Bullet: Run launcher (SITL +
renderer)**, then **bullet_app: Bullet: Run app (manual)**, then
**bullet_view: Bullet: Run view**. Auto mode remains available with
`uv run --locked bullet-app --mode auto` from a terminal.

From terminals:

```sh
cd /home/user/projects/bullet_drone
uv run --frozen --extra sitl --extra mavlink --extra render python -m examples.simulation_launcher.main

cd /home/user/projects/bullet_app
uv sync --group dev
uv run --locked bullet-app --mode auto
# Or: uv run --locked bullet-app --mode manual

cd /home/user/projects/bullet_view
uv run --locked bullet-view
```

The app waits disarmed through sensor settling. In **Auto** mode, click **Arm**
then **Start auto mission** in View. The app climbs 10 m above its starting
altitude, hovers for 10 continuous seconds, lands, and disarms. In **Manual**
mode, apply a height from 0.1 to 100 m in 0.1 m steps, click **Arm**, then
**Takeoff**. The app holds height until View sends **Land**, then descends and
disarms. The app stays ready for another flight after a normal landing.
The 0.1 m lower limit completes takeoff and landing in SITL but oscillates near
the ground; use a higher target when a steady hover matters.

View sends a heartbeat while connected. If it disappears for 3 seconds, an
armed idle app disarms and an airborne app lands. Stale SITL telemetry causes
an app fault and disarm; restart the app before another flight. Emergency
Disarm in View immediately stops motors and requires an app restart.

`--msp-host`, `--msp-port`, and `--mavlink-endpoint` select nondefault SITL
connections. This controller and its throttle tuning are for SITL only.

Run focused tests with `uv run --locked pytest -q`.
