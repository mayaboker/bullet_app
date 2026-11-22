"""Own the external SITL connections and pace View-gated flight control."""
from contextlib import ExitStack
import time

from beta_client.entities import RcChannels
from beta_client.mavlink import MavlinkTelemetry
from beta_client.msp import read_board_identifier, send_rc
from beta_client.transport import SerialTransport, TcpTransport
from loguru import logger
from pymavlink import mavutil

from .blackboard import Blackboard, StepContext
from .config import AppConfig, ConnectionType
from .ingress import TargetIngress
from .missions import Action, Command, Mode, Phase, create_mission
from .runtime_diagnostics import RuntimeDiagnostics
from .tracking_log import TrackingLog
from .view_link import ViewLink

CONTROL_CENTER = 1500
AUX_LOW = 1000
AUX_HIGH = 2000
THROTTLE_MIN = 1000
CONTROL_PERIOD_S = 0.02
TELEMETRY_RATE_HZ = 50.0
HEARTBEAT_STALE_S = 3.0
ALTITUDE_STALE_S = 1.0
ALTITUDE_STARTUP_TIMEOUT_S = 10.0
DISARM_SEND_COUNT = 3


def _send_command(transport: TcpTransport | SerialTransport, command: Command) -> None:
    channels = RcChannels((command.roll, command.pitch, command.throttle, command.yaw,
                           AUX_HIGH if command.arm else AUX_LOW, AUX_HIGH, AUX_LOW, AUX_LOW))
    send_rc(transport, channels)
    transport.drain_responses()


def _advance_automatic_start(mission, context: StepContext) -> None:
    if mission.started:
        return
    now = context.now_s
    if mission.mission_kind == 'target-hit' and not mission.sync_ready(now):
        return
    if mission.phase == Phase.DISARMED:
        mission.request(Action.ARM, now, automatic=True)
    elif mission.phase == Phase.ARMED:
        mission.request(Action.START, now, context.altitude_m, automatic=True)


def _automatic_result(mission, firmware_armed: bool) -> int | None:
    if mission.phase == Phase.FAULT and not firmware_armed:
        return 1
    if not mission.started:
        return None
    if mission.phase == Phase.DISARMED:
        return 0 if mission.result in (None, 'hit') else 1
    return None


def _confirm_sitl(transport: TcpTransport | SerialTransport) -> bool:
    """Read the Betaflight board identity without preventing startup on failure."""
    try:
        board_identifier = read_board_identifier(transport)
    except (OSError, ValueError, TimeoutError) as error:
        logger.warning('Betaflight SITL not confirmed: {}', error)
        return False
    logger.info('Betaflight board identifier: {} (SITL {})', board_identifier,
                'confirmed' if board_identifier == 'SITL' else 'not confirmed')
    return board_identifier == 'SITL'


def _open_msp(config: AppConfig) -> TcpTransport | SerialTransport:
    """Open the selected MSP transport after configuration is validated."""
    if config.msp.type is ConnectionType.TCP:
        return TcpTransport(config.msp.host, config.msp.port)
    return SerialTransport(config.msp.device, config.msp.baudrate)


def _open_mavlink(config: AppConfig) -> MavlinkTelemetry:
    """Create telemetry for a TCP endpoint or a serial device and baud rate."""
    if config.mavlink.type is ConnectionType.TCP:
        return MavlinkTelemetry(config.mavlink.endpoint)
    return MavlinkTelemetry(config.mavlink.endpoint, baud=config.mavlink.baudrate)


def _collect_inputs(mission, context: StepContext, ingress, view) -> None:
    """Apply target observations and View commands before the mission step."""
    now = context.now_s
    blackboard = context.blackboard
    if ingress is not None:
        mission.observe_timesync(blackboard.telemetry.timesync, now)
        bboxes, contact = ingress.poll()
        for bbox in bboxes:
            if mission.observe_bbox(bbox, now):
                view.publish_bbox(blackboard.visual.bbox)
        mission.observe_contact(contact)
    view.poll(mission, context)
    blackboard.view_alive = view.view_alive(now)


def _step_with_safety(mission, context: StepContext, connected_at: float) -> Command:
    """Use fresh feedback for flight; otherwise fault and send a safe command."""
    now = context.now_s
    sample = context.blackboard.telemetry
    heartbeat_fresh = sample.heartbeat is not None and now - sample.heartbeat_at <= HEARTBEAT_STALE_S
    altitude_fresh = (context.altitude_m is not None and sample.vfr_hud_at > 0
                      and now - sample.vfr_hud_at <= ALTITUDE_STALE_S)
    if not heartbeat_fresh or (not altitude_fresh and now - connected_at > ALTITUDE_STARTUP_TIMEOUT_S):
        if mission.phase != Phase.FAULT:
            mission.fault('SITL telemetry stale', now)
    if mission.phase == Phase.FAULT or not heartbeat_fresh or not altitude_fresh:
        return Command(THROTTLE_MIN, False)
    try:
        command = mission.step(context)
        if mission.mode == Mode.AUTO:
            _advance_automatic_start(mission, context)
        return command
    except ValueError as error:
        mission.fault(str(error), now)
        return Command(THROTTLE_MIN, False)


def run_mission(options) -> int:
    """Run one auto flight or serve repeated View-approved flights."""
    config = AppConfig.from_yaml(options.config).with_overrides(
        msp_host=options.msp_host, msp_port=options.msp_port,
        mavlink_endpoint=options.mavlink_endpoint)
    blackboard = Blackboard()
    blackboard.reset()
    mission = create_mission(options, blackboard)
    tracking_log = TrackingLog() if mission.mission_kind == 'target-hit' else None
    with ExitStack() as resources:
        telemetry = resources.enter_context(_open_mavlink(config))
        telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, TELEMETRY_RATE_HZ)
        telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, TELEMETRY_RATE_HZ)
        if options.mission == 'target-hit':
            telemetry.enable_timesync()
        transport = resources.enter_context(_open_msp(config))
        sitl_confirmed = _confirm_sitl(transport)
        view = resources.enter_context(ViewLink())
        view.sitl_confirmed = sitl_confirmed
        ingress = (resources.enter_context(TargetIngress(options.bbox_endpoint, options.contact_endpoint))
                   if options.mission == 'target-hit' else None)
        try:
            connected_at = time.monotonic()
            next_tick = connected_at
            diagnostics = RuntimeDiagnostics(connected_at)
            while True:
                blackboard.telemetry = telemetry.snapshot()
                context = StepContext(blackboard, time.monotonic())
                _collect_inputs(mission, context, ingress, view)
                command = _step_with_safety(mission, context, connected_at)
                _send_command(transport, command)
                if tracking_log is not None:
                    tracking_log.observe(mission, context, command)
                view.publish(mission, context)
                diagnostics.observe_events(mission, context, view, target_enabled=ingress is not None)
                if mission.mode == Mode.AUTO:
                    outcome = _automatic_result(mission, context.firmware_armed)
                    if outcome is not None:
                        logger.info('mission finished: {}', 'success' if outcome == 0 else 'failed')
                        return outcome
                diagnostics.observe_periodic(mission, context, command, view)
                next_tick += CONTROL_PERIOD_S
                remaining = next_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.monotonic()
        finally:
            for _ in range(DISARM_SEND_COUNT):
                try:
                    _send_command(transport, Command(THROTTLE_MIN, False))
                except OSError:
                    break
