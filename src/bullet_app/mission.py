"""Own the external SITL connections and pace View-gated flight control."""
from contextlib import ExitStack
import time

from beta_client.entities import RcChannels
from beta_client.mavlink import MavlinkTelemetry
from beta_client.msp import send_rc
from beta_client.transport import TcpTransport
from loguru import logger
from pymavlink import mavutil

from .missions import Command, Phase, create_mission
from .view_link import ViewLink
from .ingress import TargetIngress
from .rate_monitor import RateMonitor

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
LOG_PERIOD_S = 1.0


def _send_command(transport: TcpTransport, command: Command) -> None:
    channels = RcChannels((command.roll, command.pitch, command.throttle, command.yaw,
                           AUX_HIGH if command.arm else AUX_LOW, AUX_HIGH, AUX_LOW, AUX_LOW))
    send_rc(transport, channels)
    transport.drain_responses()


def _advance_automatic_start(mission, now: float, altitude: float) -> None:
    if mission.started:
        return
    if mission.phase == Phase.DISARMED:
        mission.request('arm', now, automatic=True)
    elif mission.phase == Phase.ARMED:
        mission.request('start', now, altitude, automatic=True)


def _automatic_result(mission, firmware_armed: bool) -> int | None:
    if mission.phase == Phase.FAULT and not firmware_armed:
        return 1
    if not mission.started:
        return None
    if mission.phase == Phase.DISARMED:
        return 0 if mission.result in (None, 'hit') else 1
    return None


def run_mission(options) -> int:
    """Run one auto flight or serve repeated View-approved flights."""
    mission = create_mission(options)
    with ExitStack() as resources:
        telemetry = resources.enter_context(MavlinkTelemetry(options.mavlink_endpoint))
        telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, TELEMETRY_RATE_HZ)
        telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, TELEMETRY_RATE_HZ)
        transport = resources.enter_context(TcpTransport(options.msp_host, options.msp_port))
        view = resources.enter_context(ViewLink())
        ingress = (resources.enter_context(TargetIngress(options.bbox_endpoint, options.contact_endpoint))
                   if options.mission == 'target-hit' else None)
        try:
            connected_at = time.monotonic()
            next_tick = connected_at
            last_log = 0.0
            previous_phase = None
            previous_result = None
            previous_navigation = None
            previous_alignment_counts = (0, 0)
            previous_estimator_counts = (0, 0, 0)
            rates = RateMonitor(connected_at)
            while True:
                sample = telemetry.snapshot()
                now = time.monotonic()
                altitude = sample.vfr_hud.alt if sample.vfr_hud is not None else None
                if ingress is not None:
                    bboxes, contact = ingress.poll()
                    for bbox in bboxes:
                        if mission.observe_bbox(bbox, now):
                            view.publish_bbox(bbox)
                    mission.observe_contact(contact)
                    mission.sample = sample
                view.poll(mission, now, altitude)
                firmware_armed = bool(sample.heartbeat is not None and
                                      sample.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                heartbeat_fresh = sample.heartbeat is not None and now - sample.heartbeat_at <= HEARTBEAT_STALE_S
                altitude_fresh = (altitude is not None and sample.vfr_hud_at > 0
                                  and now - sample.vfr_hud_at <= ALTITUDE_STALE_S)
                if not heartbeat_fresh or (not altitude_fresh and now - connected_at > ALTITUDE_STARTUP_TIMEOUT_S):
                    if mission.phase != Phase.FAULT:
                        mission.fault('SITL telemetry stale', now)
                if mission.phase != Phase.FAULT and heartbeat_fresh and altitude_fresh:
                    try:
                        command = mission.step(now, altitude, sample.vfr_hud_at,
                                               firmware_armed, view.view_alive(now))
                        if options.mode == 'auto':
                            _advance_automatic_start(mission, now, altitude)
                    except ValueError as error:
                        mission.fault(str(error), now)
                        command = Command(THROTTLE_MIN, False)
                else:
                    command = Command(THROTTLE_MIN, False)
                _send_command(transport, command)
                view.publish(mission, sample, now, firmware_armed if heartbeat_fresh else False)
                for name, event, hz in rates.check(now, sample.attitude_hz, sample.vfr_hud_hz):
                    if event == 'low':
                        logger.warning('{} rate low: {:.1f} Hz', name, hz)
                    else:
                        logger.info('{} rate recovered: {:.1f} Hz', name, hz)
                    view.publish_rate_alert(name, event, hz)
                if mission.phase != previous_phase:
                    logger.info('state: {} ({})', mission.phase.value, options.mode)
                    previous_phase = mission.phase
                if mission.result is not None and mission.result != previous_result:
                    if mission.result == 'hit':
                        logger.info('mission result: {}', mission.result)
                    else:
                        logger.warning('mission result: {}', mission.result)
                    previous_result = mission.result
                navigation = getattr(mission, 'last_navigation_result', None)
                if navigation is not None:
                    current_navigation = (navigation.state, navigation.reason)
                    if current_navigation != previous_navigation:
                        logger.info('LOS state={} reason={} vision={}', navigation.state.value,
                                    navigation.reason, navigation.diagnostics.vision_reason)
                        previous_navigation = current_navigation
                if options.mode == 'auto':
                    outcome = _automatic_result(mission, firmware_armed)
                    if outcome is not None:
                        logger.info('mission finished: {}', 'success' if outcome == 0 else 'failed')
                        return outcome
                if now - last_log >= LOG_PERIOD_S and altitude is not None:
                    datum = mission.ground_altitude_m if mission.ground_altitude_m is not None else altitude
                    logger.info('altitude={:.2f} m speed={:.2f} m/s throttle={} armed={} view={}',
                                altitude - datum, mission.controller.speed, command.throttle,
                                firmware_armed, view.view_alive(now))
                    logger.info('SITL rates ATTITUDE={:.1f} Hz VFR_HUD={:.1f} Hz, ages attitude={:.3f} s altitude={:.3f} s',
                                sample.attitude_hz, sample.vfr_hud_hz,
                                now - sample.attitude_at if sample.attitude_at else float('nan'),
                                now - sample.vfr_hud_at if sample.vfr_hud_at else float('nan'))
                    if (mission.mission_kind == 'target-hit' and sample.attitude is not None
                            and mission.boot_offset_s is not None):
                        candidate_offset = sample.attitude_at - sample.attitude.time_boot_ms / 1000
                        logger.info('ATTITUDE clock source_age={:.3f} s offset_gap={:.3f} s boot_ms={}',
                                    now - mission.sensor_estimator.attitude_latest.timestamp_s
                                    if mission.sensor_estimator.attitude_latest is not None else float('nan'),
                                    candidate_offset - mission.boot_offset_s, sample.attitude.time_boot_ms)
                    navigation_input = getattr(mission, 'last_navigation_input', None)
                    if mission.phase in (Phase.LOS, Phase.COMMIT) and navigation is not None and navigation_input is not None:
                        target, attitude = navigation_input.target, navigation_input.attitude
                        counts = (navigation.diagnostics.alignment_waits,
                                  navigation.diagnostics.alignment_replacements)
                        logger.info('LOS timing image_age={} attitude_age={} image_minus_attitude={} '
                                    'aligned_age={} waits={} replacements={} vision={}',
                                    None if target is None else round(now - target.timestamp_s, 3),
                                    None if attitude is None else round(now - attitude.timestamp_s, 3),
                                    None if target is None or attitude is None else round(target.timestamp_s - attitude.timestamp_s, 3),
                                    None if navigation.diagnostics.image_age_s is None else round(navigation.diagnostics.image_age_s, 3),
                                    counts[0] - previous_alignment_counts[0],
                                    counts[1] - previous_alignment_counts[1],
                                    navigation.diagnostics.vision_reason)
                        previous_alignment_counts = counts
                        estimator = mission.sensor_estimator
                        estimator_counts = (estimator.skipped_ticks, estimator.expired_boxes,
                                            estimator.overflow_boxes)
                        logger.info('LOS estimator attitude_source_age={} altitude_source_age={} '
                                    'attitude_prediction_age={} altitude_prediction_age={} '
                                    'queue_depth={} skipped_ticks={} expired_boxes={} overflow_boxes={}',
                                    None if estimator.attitude_latest is None else round(now - estimator.attitude_latest.timestamp_s, 3),
                                    None if estimator.barometer_latest is None else round(now - estimator.barometer_latest.timestamp_s, 3),
                                    None if estimator.last_attitude_prediction_age_s is None else round(estimator.last_attitude_prediction_age_s, 3),
                                    None if estimator.last_altitude_prediction_age_s is None else round(estimator.last_altitude_prediction_age_s, 3),
                                    estimator.queue_depth,
                                    estimator_counts[0] - previous_estimator_counts[0],
                                    estimator_counts[1] - previous_estimator_counts[1],
                                    estimator_counts[2] - previous_estimator_counts[2])
                        previous_estimator_counts = estimator_counts
                    last_log = now
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
