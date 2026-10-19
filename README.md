# Bullet app

This project is a member of the [Bullet workspace](../README.md). All six
projects share `../.venv` and `../uv.lock`; run uv commands with the
workspace flags below to retain every project and optional dependency.

Standalone Python 3.12+ uv application, intended to own the drone application
state machine. This version only runs the camera → red detector → bounding-box
publisher pipeline. Flight commands and state transitions are deferred.

Keep `bullet_app`, `los_tracking`, `bullet_msgs`, `mcap_bagger`, and `bullet_render`
as sibling folders under `~/projects/bullet`.

```bash
cd ~/projects/bullet/bullet_app
uv sync --locked --all-packages --all-extras --all-groups
uv run --locked --all-packages --all-extras --all-groups bullet-app --transport-config ../bullet_render/configs/transport.json --debug
# Equivalent example entry point:
uv run --locked --all-packages --all-extras --all-groups python examples/live_los.py --transport-config ../bullet_render/configs/transport.json --debug
```

The Bullet workspace includes **Bullet: App (debug)**. **Bullet: Physics + render
+ app** and **Bullet: Run all** launch it with Godot, physics, and the MCAP
logger. Click Start in Godot to produce images and start episode recording.
The debug application provides the camera and bounding-box preview.

The receiver moved here from `los_tracking`; it uses `bullet_msgs.FrameReader`
for exact topic matching, SHM validation, private BGR copies, duplicate/stale
skips, row padding, and publisher restarts. `los_tracking` now owns detection
only. The former live example also moved here to avoid a dependency cycle.

## Input and output

`--transport-config` is required. `--topic` defaults to `camera/front/image`.
`--bbox-endpoint` defaults to `tcp://127.0.0.1:5558`; the app binds a PUB socket
and MCAP connects. It must differ from the renderer/physics/control endpoints.
When changing this port, also change the bbox topic's `endpoint` in
`../bullet_render/configs/recorder.yaml`.

Every processed frame emits exactly two ZMQ parts: UTF-8 topic
`tracking/front/bbox` and a JSON payload using `bullet_msgs:bbox.json`:

```json
{"schema_version":1,"image_topic":"camera/front/image","session":"0123456789abcdef0123456789abcdef","sequence":42,"capture_ns":123456789,"bbox":{"x":100,"y":80,"width":40,"height":30}}
```

Position is the bounding box's top-left pixel, x right and y down, with width and
height in pixels. It is not a 3D target pose. `bbox: null` explicitly reports no
red target in that frame. Session, sequence, and capture_ns come unchanged from
the source image descriptor. capture_ns is monotonic time, not Unix time. MCAP
uses arrival time; source identity can match a bbox to a descriptor but recorded
RawImage messages do not preserve that identity, so exact MCAP image association
is not guaranteed.

No new frame means no new result. The app waits for the renderer and resumes on
restart. Debug mode shows yellow boxes and Waiting/Live/Stale status (stale after
one second without a new image). A no-target frame clears the box. Q, Esc,
window close, or Ctrl+C releases sockets and mappings. Normal mode opens no
window. Malformed input or bind failures exit with a diagnostic and nonzero code.

Publication uses nonblocking PUB/SUB with a 64-message send high-water mark.
Disconnected/slow subscribers and subscription startup can lose results; there
are no delivery acknowledgments or retries. MCAP's existing episode lifecycle
still controls when results are recorded. HSV detection selects the largest red
object and can select unrelated red objects. SHM requires the same Linux host/user
as Godot; the debug view requires a graphical display.

## Checks

```bash
uv run --locked --all-packages --all-extras --all-groups pytest -q
```

Tests include TCP/SHM reception, debug/non-GUI paths, JSON contracts, and a real
image-to-detection-to-MCAP round trip using synthetic images.
