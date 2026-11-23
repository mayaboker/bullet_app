"""Own the external SITL connections and pace View-gated flight control."""
from contextlib import ExitStack
from concurrent.futures import ThreadPoolExecutor
import time

from beta_client.entities import RcChannels
from beta_client.mavlink import MavlinkTelemetry
from beta_client.msp import read_board_identifier, send_rc
from beta_client.transport import SerialTransport, TcpTransport
from loguru import logger
from pymavlink import mavutil

from .blackboard import Blackboard, StepContext
from .config import AppConfig, ConnectionType
from .tracker import TrackerSubscriber
from .missions import Action, Command, Mode, Phase, create_mission
from .runtime_diagnostics import RuntimeDiagnostics
from .telemetry_publisher import TelemetryPublisher, needs_timesync
from .view_link import ViewLink

CONTROL_CENTER = 1500
AUX_LOW = 1000
AUX_HIGH = 2000
THROTTLE_MIN = 1000
CONTROL_PERIOD_S = 0.02
TELEMETRY_RATE_HZ = 50.0
GPS_TOPICS = (
    ('telemetry/gps_raw_int', mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT),
    ('telemetry/global_position_int', mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT),
)
HEARTBEAT_STALE_S = 3.0
ALTITUDE_STALE_S = 1.0
ALTITUDE_STARTUP_TIMEOUT_S = 10.0
DISARM_SEND_COUNT = 3
RECONNECT_READY_TIMEOUT_S = 10.0


def _send_command(transport: TcpTransport | SerialTransport, command: Command):
    """Send one MSP RC frame and return the exact channels and local send time."""
    channels = RcChannels((command.roll, command.pitch, command.throttle, command.yaw,
                           AUX_HIGH if command.arm else AUX_LOW, AUX_HIGH, AUX_LOW, AUX_LOW))
    send_rc(transport, channels)
    sent_monotonic_ns = time.monotonic_ns()
    transport.drain_responses()
    return channels, sent_monotonic_ns


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


def _request_telemetry_rates(telemetry: MavlinkTelemetry, config: AppConfig) -> None:
    """Request mission feedback and any configured GPS recording streams."""
    telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, TELEMETRY_RATE_HZ)
    telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, TELEMETRY_RATE_HZ)
    rates = dict(config.telemetry_publisher.topics)
    for topic, message_id in GPS_TOPICS:
        if rate := rates.get(topic, 0):
            telemetry.set_message_rate(message_id, rate)


def _open_tracker(options, resources: ExitStack):
    """Open the target tracker when the selected mission uses boxes."""
    if options.mission != 'target-hit':
        return None
    return resources.enter_context(TrackerSubscriber(options.bbox_endpoint))


def _open_simulation(options, config: AppConfig, resources: ExitStack):
    """Open simulator contact only when enabled for target-hit."""
    if options.mission != 'target-hit' or not config.simulation.enabled:
        return None
    from .simulation import SimulationContactSubscriber
    return resources.enter_context(SimulationContactSubscriber(config.simulation.contact_endpoint))


def _collect_inputs(mission, context: StepContext, tracker, simulation, view,
                    restart_handler=None) -> None:
    """Apply target observations and View commands before the mission step."""
    now = context.now_s
    blackboard = context.blackboard
    if tracker is not None:
        mission.observe_timesync(blackboard.telemetry.timesync, now)
        for bbox in tracker.poll():
            if mission.observe_bbox(bbox, now):
                view.publish_bbox(blackboard.visual.bbox)
    if simulation is not None:
        mission.observe_simulation_contact(simulation.poll())
    if restart_handler is None:
        view.poll(mission, context)
    else:
        view.poll(mission, context, restart_handler)
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


# application entry point for the standalone flight mission
def run_mission(options) -> int:
    """Run one auto flight or serve repeated View-approved flights."""
    config = AppConfig.from_yaml(options.config).with_overrides(
        msp_host=options.msp_host, msp_port=options.msp_port,
        mavlink_endpoint=options.mavlink_endpoint,
        contact_endpoint=options.contact_endpoint)
    if getattr(options, 'mode', 'auto') == 'manual':
        return _run_manual(options, config)
    blackboard = _init_blackboard()
    mission = create_mission(options, blackboard)

    with ExitStack() as resources:
        telemetry = resources.enter_context(_open_mavlink(config))
        _request_telemetry_rates(telemetry, config)
        if options.mission == 'target-hit' or needs_timesync(config.telemetry_publisher):
            telemetry.enable_timesync()

        transport = resources.enter_context(_open_msp(config))
        sitl_confirmed = _confirm_sitl(transport)

        view = resources.enter_context(ViewLink())
        view.sitl_confirmed = sitl_confirmed

        tracker = _open_tracker(options, resources)
        simulation = _open_simulation(options, config, resources)
        publisher = (resources.enter_context(TelemetryPublisher(config.telemetry_publisher, blackboard))
                     if config.telemetry_publisher.enabled else None)
        try:
            connected_at = time.monotonic()
            next_tick = connected_at
            diagnostics = RuntimeDiagnostics(connected_at)
            while True:
                blackboard.telemetry = telemetry.snapshot()
                context = StepContext(blackboard, time.monotonic())
                _collect_inputs(mission, context, tracker, simulation, view)
                command = _step_with_safety(mission, context, connected_at)
                sent = _send_command(transport, command)
                if publisher is not None and sent is not None:
                    publisher.enqueue_rc(sent[0], mission.phase.value, sent[1])
                mission.on_command_sent(context, command)
                view.publish(mission, context)
                diagnostics.observe_events(mission, context, view, target_enabled=tracker is not None)
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
                    sent = _send_command(transport, Command(THROTTLE_MIN, False))
                    if publisher is not None and sent is not None:
                        publisher.enqueue_rc(sent[0], mission.phase.value, sent[1])
                except OSError:
                    break


def _init_blackboard():
    blackboard = Blackboard()
    blackboard.reset()
    return blackboard


def _connect_flight(config: AppConfig):
    """Open blocking Betaflight connections away from the View command loop."""
    resources = ExitStack()
    try:
        telemetry = resources.enter_context(_open_mavlink(config))
        _request_telemetry_rates(telemetry, config)
        if needs_timesync(config.telemetry_publisher):
            telemetry.enable_timesync()
        transport = resources.enter_context(_open_msp(config))
        sitl_confirmed = _confirm_sitl(transport)
        if config.view_restart_enabled and not sitl_confirmed:
            raise ValueError('Restart-enabled connection did not identify as SITL')
        return resources, telemetry, transport, sitl_confirmed
    except BaseException:
        resources.close()
        raise


def _manual_config(options):
    return AppConfig.from_yaml(options.config).with_overrides(
        msp_host=options.msp_host, msp_port=options.msp_port,
        mavlink_endpoint=options.mavlink_endpoint,
        contact_endpoint=options.contact_endpoint)


def _run_manual(options, initial_config: AppConfig) -> int:
    """Keep View connected while individual disarmed SITL sessions come and go."""
    blackboard = _init_blackboard()
    session = None
    pending_restart = False
    config = initial_config
    connect_started = time.monotonic()
    with ViewLink() as view, ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(_connect_flight, config)
        view.app_state = 'restarting' if config.view_restart_enabled else 'disabled'
        view.app_epoch = 0
        next_tick = time.monotonic()

        def request_restart():
            nonlocal config, future, pending_restart, connect_started, session
            if not initial_config.view_restart_enabled or options.mode != 'manual':
                raise ValueError('Restart disabled')
            if (initial_config.msp.type is not ConnectionType.TCP or
                    initial_config.mavlink.type is not ConnectionType.TCP):
                raise ValueError('Restart requires SITL TCP connections')
            if session is not None and not _confirmed_disarmed(blackboard):
                raise ValueError('Fresh disarmed telemetry required')
            if future is not None or view.app_state == 'restarting':
                raise ValueError('Restart already in progress')
            try:
                reloaded = _manual_config(options)
            except Exception as error:
                raise ValueError(f'Restart config invalid: {error}') from error
            if (not reloaded.view_restart_enabled or
                    reloaded.msp.type is not ConnectionType.TCP or
                    reloaded.mavlink.type is not ConnectionType.TCP):
                raise ValueError('Restart requires enabled SITL TCP config')
            config = reloaded
            if session is not None:
                _close_manual_session(session)
                session = None
            blackboard.reset()
            view.datum_m = None
            view.last_phase = view.last_result = None
            view.sitl_confirmed = False
            view.app_state = 'restarting'
            connect_started = time.monotonic()
            future = executor.submit(_connect_flight, config)
            pending_restart = True

        def session_lost(reason):
            nonlocal session, pending_restart
            logger.warning('Flight connection lost: {}', reason)
            if session is not None:
                _close_manual_session(session)
                session = None
            view.app_state = ('restart_failed' if pending_restart else
                              'restart_required' if config.view_restart_enabled else 'disabled')
            view.sitl_confirmed = False
            blackboard.reset()
            if pending_restart:
                view.finish_restart(False, str(reason))
                pending_restart = False

        try:
            while True:
                now = time.monotonic()
                if future is not None and future.done():
                    opened_resources = None
                    try:
                        resources, telemetry, transport, sitl_confirmed = future.result()
                        opened_resources = resources
                        blackboard.reset()
                        mission = create_mission(options, blackboard)
                        if options.mission == 'target-hit':
                            telemetry.enable_timesync()
                        tracker = _open_tracker(options, resources)
                        simulation = _open_simulation(options, config, resources)
                        publisher = (resources.enter_context(TelemetryPublisher(config.telemetry_publisher, blackboard))
                                     if config.telemetry_publisher.enabled else None)
                        session = (resources, telemetry, transport, mission, tracker, simulation,
                                   publisher, RuntimeDiagnostics(now), now)
                        view.sitl_confirmed = sitl_confirmed
                    except Exception as error:
                        if opened_resources is not None:
                            opened_resources.close()
                        logger.error('Flight session open failed: {}', error)
                        view.app_state = 'restart_failed' if config.view_restart_enabled else 'disabled'
                        if pending_restart:
                            view.finish_restart(False, str(error))
                            pending_restart = False
                    finally:
                        future = None
                if session is not None:
                    resources, telemetry, transport, mission, tracker, simulation, publisher, diagnostics, connected_at = session
                    try:
                        blackboard.telemetry = telemetry.snapshot()
                    except (RuntimeError, OSError, ConnectionError, TimeoutError) as error:
                        session_lost(error)
                if session is not None:
                    resources, telemetry, transport, mission, tracker, simulation, publisher, diagnostics, connected_at = session
                    context = StepContext(blackboard, now)
                    heartbeat = blackboard.telemetry.heartbeat
                    heartbeat_fresh = (heartbeat is not None and
                                       now - blackboard.telemetry.heartbeat_at <= HEARTBEAT_STALE_S)
                    if heartbeat_fresh and context.firmware_armed and view.app_state == 'restarting':
                        view.app_state = 'restart_failed'
                        if pending_restart:
                            view.finish_restart(False, 'Vehicle is armed after reconnect')
                            pending_restart = False
                        _close_manual_session(session)
                        session = None
                        blackboard.reset()
                    elif heartbeat_fresh and not context.firmware_armed and view.app_state == 'restarting':
                        view.app_state = 'ready' if config.view_restart_enabled else 'disabled'
                        view.app_epoch = (view.app_epoch + 1) % (1 << 24)
                        if pending_restart:
                            view.finish_restart(True)
                            pending_restart = False
                    elif view.app_state == 'restarting' and now - connect_started > RECONNECT_READY_TIMEOUT_S:
                        view.app_state = 'restart_failed'
                        if pending_restart:
                            view.finish_restart(False, 'Disarmed telemetry not ready')
                            pending_restart = False
                    if session is not None and not heartbeat_fresh and now - connected_at > HEARTBEAT_STALE_S:
                        session_lost('Flight telemetry stale')
                    elif session is not None:
                        try:
                            _collect_inputs(mission, context, tracker, simulation, view, request_restart)
                            if session is not None:
                                command = _step_with_safety(mission, context, connected_at)
                                sent = _send_command(transport, command)
                                if publisher is not None and sent is not None:
                                    publisher.enqueue_rc(sent[0], mission.phase.value, sent[1])
                                mission.on_command_sent(context, command)
                                view.publish(mission, context)
                                diagnostics.observe_events(mission, context, view, target_enabled=tracker is not None)
                                diagnostics.observe_periodic(mission, context, command, view)
                        except (RuntimeError, OSError, ConnectionError, TimeoutError) as error:
                            session_lost(error)
                if session is None:
                    context = StepContext(blackboard, now)
                    view.poll(None, context, request_restart)
                    blackboard.view_alive = view.view_alive(now)
                view.publish_app_status(now, session_ready=session is not None)
                next_tick += CONTROL_PERIOD_S
                remaining = next_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.monotonic()
        finally:
            if session is not None:
                _close_manual_session(session)


def _confirmed_disarmed(blackboard) -> bool:
    sample = blackboard.telemetry
    return (sample.heartbeat is not None and
            time.monotonic() - sample.heartbeat_at <= HEARTBEAT_STALE_S and
            not StepContext(blackboard, time.monotonic()).firmware_armed)


def _close_manual_session(session) -> None:
    resources, _, transport, mission, _, _, publisher, *_ = session
    for _ in range(DISARM_SEND_COUNT):
        try:
            sent = _send_command(transport, Command(THROTTLE_MIN, False))
            if publisher is not None and sent is not None:
                publisher.enqueue_rc(sent[0], mission.phase.value, sent[1])
        except (OSError, ConnectionError, TimeoutError):
            break
    resources.close()
