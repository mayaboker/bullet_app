"""Loopback MAVLink bridge between Bullet App and Bullet View."""
from math import isfinite
from time import monotonic

from pymavlink import mavutil
from pymavlink.dialects.v20 import common
from loguru import logger
from bullet_simulation_protocol.view_bbox import MAX_PAYLOAD_BYTES, MESSAGE_TYPE, encode_bbox

from .missions import Action, HoverLandMission, Mode, TargetMission, PHASE_IDS, Phase
from .missions.base import Readiness
from .blackboard import StepContext

APP_SYSTEM_ID = 1
APP_COMPONENT_ID = 1
VIEW_SYSTEM_ID = 255
VIEW_COMPONENT_ID = 190
VIEW_ENDPOINT = 'udpout:127.0.0.1:14550'
VIEW_TIMEOUT_S = 3.0
HEARTBEAT_PERIOD_S = 1.0
SITL_CONFIRMED_CUSTOM_MODE = 1 << 0
STATE_PERIOD_S = 0.1
TELEMETRY_PERIOD_S = 0.05
RATE_PERIOD_S = 1.0
ARM_COMMAND = 400
START_COMMAND = 300
HEIGHT_COMMAND = 31012
LAND_COMMAND = 21
FORCE_DISARM_MAGIC = 21196
ACTION_IDS = {Action.ARM: 1, Action.DISARM: 2, Action.EMERGENCY: 3,
              Action.START: 4, Action.HEIGHT: 5, None: 0, Action.LAND: 6}


def _mavlink2(connection) -> None:
    connection.source_system = APP_SYSTEM_ID
    connection.source_component = APP_COMPONENT_ID
    connection.mav = common.MAVLink(connection, srcSystem=APP_SYSTEM_ID, srcComponent=APP_COMPONENT_ID)
    connection.mav.robust_parsing = connection.robust_parsing
    connection.WIRE_PROTOCOL_VERSION = '2.0'
    connection.first_byte = False


def _command(message) -> tuple[Action, float | None]:
    values = [getattr(message, f'param{index}') for index in range(1, 8)]
    if not all(isfinite(value) for value in values):
        raise ValueError('Nonfinite flight parameter')
    first, second, *rest = values
    if any(value != 0 for value in rest):
        raise ValueError('Unsupported flight parameter')
    if message.command == ARM_COMMAND:
        if first == 1 and second == 0:
            return Action.ARM, None
        if first == 0 and second == 0:
            return Action.DISARM, None
        if first == 0 and second == FORCE_DISARM_MAGIC:
            return Action.EMERGENCY, None
    elif message.command == START_COMMAND and first == second == 0:
        return Action.START, None
    elif message.command == HEIGHT_COMMAND and second == 0:
        return Action.HEIGHT, first
    elif message.command == LAND_COMMAND and first == second == 0:
        return Action.LAND, None
    raise ValueError('Unsupported flight command')


class ViewLink:
    def __init__(self, endpoint=VIEW_ENDPOINT, connection_factory=mavutil.mavlink_connection):
        self.endpoint = endpoint
        self.connection_factory = connection_factory
        self.connection = None
        self.last_view_at: float | None = None
        self.last_heartbeat_at = self.last_state_at = self.last_telemetry_at = 0.0
        self.last_rate_at = 0.0
        self.last_phase: Phase | None = None
        self.last_result = None
        self.datum_m: float | None = None
        self.sitl_confirmed = False

    def __enter__(self):
        try:
            self.connection = self.connection_factory(self.endpoint,
                                                      source_system=APP_SYSTEM_ID, source_component=APP_COMPONENT_ID)
            _mavlink2(self.connection)
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        if self.connection is not None:
            self.connection.close()

    def view_alive(self, now: float) -> bool:
        return self.last_view_at is not None and 0 <= now - self.last_view_at <= VIEW_TIMEOUT_S

    def publish_bbox(self, packet) -> None:
        if 'fid' not in packet:
            return
        payload = encode_bbox(packet['fid'], packet['bbox']).ljust(MAX_PAYLOAD_BYTES, b'\0')
        self.connection.mav.v2_extension_send(0, VIEW_SYSTEM_ID, VIEW_COMPONENT_ID,
                                               MESSAGE_TYPE, payload)

    def publish_rate_alert(self, name: str, event: str, hz: float) -> None:
        description = f'{name} rate {"low" if event == "low" else "recovered"}: {hz:.1f} Hz'
        severity = 4 if event == 'low' else 6
        self.connection.mav.statustext_send(severity, description.encode('ascii')[:50])

    def poll(self, mission: HoverLandMission | TargetMission, context: StepContext) -> None:
        now = context.now_s
        for _ in range(32):
            message = self.connection.recv_match(type=['HEARTBEAT', 'COMMAND_LONG'], blocking=False)
            if message is None:
                break
            if (message.get_srcSystem(), message.get_srcComponent()) != (VIEW_SYSTEM_ID, VIEW_COMPONENT_ID):
                continue
            if message.get_type() == 'HEARTBEAT':
                self.last_view_at = now
                continue
            if (message.target_system, message.target_component) != (APP_SYSTEM_ID, APP_COMPONENT_ID):
                continue
            action = None
            try:
                action, value = _command(message)
                if not self.view_alive(now):
                    raise ValueError('Bullet View heartbeat is stale')
                if action == Action.HEIGHT:
                    mission.request_height(value)
                else:
                    mission.request(action, now, context.altitude_m, value)
            except ValueError as error:
                result = mavutil.mavlink.MAV_RESULT_DENIED
                name = action.value if action is not None else 'invalid'
                logger.warning('View command {} denied: {}', name, error)
                self.connection.mav.statustext_send(4, f'{name} denied: {error}'.encode('utf-8')[:50])
            else:
                result = mavutil.mavlink.MAV_RESULT_ACCEPTED
            self.connection.mav.command_ack_send(message.command, result, 0, ACTION_IDS[action],
                                                 VIEW_SYSTEM_ID, VIEW_COMPONENT_ID)

    def publish(self, mission: HoverLandMission | TargetMission, context: StepContext) -> None:
        now = context.now_s
        sample = context.blackboard.telemetry
        firmware_armed = (context.firmware_armed and sample.heartbeat is not None
                          and now - sample.heartbeat_at <= 3.0)
        elapsed_ms = int(now * 1000) % 2**32
        elapsed_us = int(now * 1_000_000) % 2**64
        mav = self.connection.mav
        if now - self.last_rate_at >= RATE_PERIOD_S:
            for name, value in (('ATT_HZ', getattr(sample, 'attitude_hz', None)),
                                ('ALT_HZ', getattr(sample, 'vfr_hud_hz', None))):
                if value is not None and isfinite(value):
                    mav.named_value_float_send(elapsed_ms, name.encode('ascii'), value)
            self.last_rate_at = now
        if now - self.last_heartbeat_at >= HEARTBEAT_PERIOD_S:
            mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if firmware_armed else 0
            status = (mavutil.mavlink.MAV_STATE_ACTIVE if mission.readiness == Readiness.FLYING
                      else mavutil.mavlink.MAV_STATE_STANDBY)
            mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                               mavutil.mavlink.MAV_AUTOPILOT_GENERIC, mode,
                               SITL_CONFIRMED_CUSTOM_MODE if self.sitl_confirmed else 0, status, 3)
            self.last_heartbeat_at = now
        if now - self.last_state_at >= STATE_PERIOD_S:
            values = (('ARM_MODE', float(firmware_armed)),
                      ('ARM_VALID', float(sample.heartbeat is not None and now - sample.heartbeat_at <= 3.0)),
                      ('MIS_READY', float(mission.readiness)), ('TAKEOFF_H', mission.height_m),
                      ('HOVER_S', mission.hover_duration_s),
                      ('APP_MODE', float(mission.mode == Mode.MANUAL)),
                      ('APP_PHASE', float(PHASE_IDS[mission.phase])),
                      ('APP_MSN', float({'hover-land': 1, 'target-hit': 2}[mission.mission_kind])),
                      ('APP_RESULT', 0.0 if mission.result is None else 1.0 if mission.result == 'hit' else 2.0),
                      ('VIS_READY', float(mission.vision_alive(now))))
            for name, value in values:
                mav.named_value_float_send(elapsed_ms, name.encode('ascii'), value)
            self.last_state_at = now
        if mission.phase != self.last_phase:
            self.last_phase = mission.phase
            description = mission.fault_reason if mission.phase == Phase.FAULT else mission.phase.value
            severity = 3 if mission.phase == Phase.FAULT else 6
            mav.statustext_send(severity, description.encode('utf-8')[:50])
        if mission.result is not None and mission.result != self.last_result:
            self.last_result = mission.result
            mav.statustext_send(6 if mission.result == 'hit' else 4,
                                ('Target result: ' + mission.result).encode('utf-8')[:50])
        if now - self.last_telemetry_at >= TELEMETRY_PERIOD_S:
            attitude = sample.attitude
            if attitude is not None and now - sample.attitude_at <= 1.0:
                mav.attitude_send(elapsed_ms, attitude.roll, attitude.pitch, attitude.yaw,
                                  attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed)
            altitude = sample.vfr_hud
            if altitude is not None and now - sample.vfr_hud_at <= 1.0 and isfinite(altitude.alt):
                if self.datum_m is None:
                    self.datum_m = altitude.alt
                datum = mission.ground_altitude_m if mission.ground_altitude_m is not None else self.datum_m
                mav.altitude_send(elapsed_us, float('nan'), float('nan'), altitude.alt - datum,
                                  float('nan'), float('nan'), float('nan'))
            self.last_telemetry_at = now
