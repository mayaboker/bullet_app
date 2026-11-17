"""Own the external SITL connections and pace View-gated flight control."""
from contextlib import ExitStack
import time

from beta_client.entities import RcChannels
from beta_client.mavlink import MavlinkTelemetry
from beta_client.msp import send_rc
from beta_client.transport import TcpTransport
from loguru import logger
from pymavlink import mavutil

from .flight import Command, FlightMission, Phase
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
LOG_PERIOD_S = 1.0


def _send_command(transport: TcpTransport, command: Command) -> None:
    channels = RcChannels((CONTROL_CENTER, CONTROL_CENTER, command.throttle, CONTROL_CENTER,
                           AUX_HIGH if command.arm else AUX_LOW, AUX_HIGH, AUX_LOW, AUX_LOW))
    send_rc(transport, channels)
    transport.drain_responses()


def run_mission(options) -> int:
    """Keep App available for repeated flights; never own SITL or renderer."""
    mission = FlightMission(options.mode)
    with ExitStack() as resources:
        telemetry = resources.enter_context(MavlinkTelemetry(options.mavlink_endpoint))
        telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, TELEMETRY_RATE_HZ)
        telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, TELEMETRY_RATE_HZ)
        transport = resources.enter_context(TcpTransport(options.msp_host, options.msp_port))
        view = resources.enter_context(ViewLink())
        try:
            connected_at = time.monotonic()
            next_tick = connected_at
            last_log = 0.0
            previous_phase = None
            while True:
                sample = telemetry.snapshot()
                now = time.monotonic()
                altitude = sample.vfr_hud.alt if sample.vfr_hud is not None else None
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
                    except ValueError as error:
                        mission.fault(str(error), now)
                        command = Command(THROTTLE_MIN, False)
                else:
                    command = Command(THROTTLE_MIN, False)
                _send_command(transport, command)
                view.publish(mission, sample, now, firmware_armed if heartbeat_fresh else False)
                if mission.phase != previous_phase:
                    logger.info('state: {} ({})', mission.phase.value, options.mode)
                    previous_phase = mission.phase
                if now - last_log >= LOG_PERIOD_S and altitude is not None:
                    datum = mission.ground_altitude_m if mission.ground_altitude_m is not None else altitude
                    logger.info('altitude={:.2f} m speed={:.2f} m/s throttle={} armed={} view={}',
                                altitude - datum, mission.controller.speed, command.throttle,
                                firmware_armed, view.view_alive(now))
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
