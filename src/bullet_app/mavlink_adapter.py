"""Publish latest mission feedback as standard MAVLink telemetry."""
import time
from math import isfinite
from threading import Event
from queue import Queue, Empty, Full
from loguru import logger

from los_tracking.geometry import euler
from pymavlink import mavutil
from bullet_msgs.status_messages import (MAV_SEVERITY_ERROR, STATUS_MESSAGE_BY_KEY,
                                         status_text_chunks)
from bullet_msgs.mavlink_control import (RELOAD_PROFILE_ACK_ACCEPTED, RELOAD_PROFILE_ACK_DENIED,
                                         RELOAD_PROFILE_COMMAND, RELOAD_PROFILE_PARAMETER,
                                         RECORDING_CONTROL_COMMAND)
from bullet_msgs.recording_control import DEFAULT_STATE_ENDPOINT, DEFAULT_STATUS_ENDPOINT
from .recording_control import RecordingBridge
from bullet_msgs.mavlink_control import (RECORDING_NAMES, RECORDING_STATES,
    RECORDING_AUTH_NONE, RECORDING_AUTH_MAVLINK, RECORDING_AUTH_GODOT)
from bullet_msgs.recording_control import STATE_TIMEOUT_S
from bullet_msgs.flight_control import FLIGHT_COMMANDS, FLIGHT_NAMES, STATE_STALE_S, parse_command
from .flight_control import FlightControl
from bullet_msgs.parameters import use_mavlink2
from .parameters import ParameterService
from .status import format_status, status_key


HEARTBEAT_PERIOD_S = 1.
TELEMETRY_PERIOD_S = .05
FEEDBACK_STALE_S = .2
SCHEDULER_POLL_S = .01
SYSTEM_ID = 1
COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
UNKNOWN_FLOAT = float('nan')
BBOX_NAMED_VALUES = ('BBOX_VALID', 'BBOX_X', 'BBOX_Y', 'BBOX_W', 'BBOX_H')
RANGEFINDER_MIN_DISTANCE_CM = 1
RANGEFINDER_MAX_DISTANCE_CM = 5000
RANGEFINDER_INVALID_CM = 65535
RELOAD_REPLY_CAPACITY = 64


def status_image_sequence(status):
    """Prefer the navigator's processed observation, then same-episode ingress.

    Processed observations are episode-filtered by Mission. The latest received
    image can be foreign while a scenario resets, so its identity is checked.
    """
    diagnostics = status.get('diagnostics') or {}
    processed = diagnostics.get('processed_image_sequence')
    if type(processed) is int and processed >= 0:
        return processed
    latest = diagnostics.get('latest_image') or {}
    source = latest.get('source') or {}
    if not all(isinstance(status.get(key), str) and status[key]
               and source.get(key) == status[key] for key in ('physics_id', 'episode_id')):
        return None
    sequence = latest.get('sequence')
    return sequence if type(sequence) is int and sequence >= 0 else None


def mavlink_attitude(orientation_xyzw):
    """Convert Bullet FLU/Z-up XYZW orientation to MAVLink FRD/NED Euler angles."""
    x, y, z, w = orientation_xyzw
    return euler((x, -y, -z, w))


class MavlinkAdapter:
    """Own MAVLink telemetry/control connections and read blackboard feedback."""
    def __init__(self, endpoint, blackboard, stop, *, control_endpoint=None,
                 connection_factory=mavutil.mavlink_connection,
                 clock=time.monotonic, parameter_config=None, recording_control_source=None,
                 recording_state_endpoint=DEFAULT_STATE_ENDPOINT,
                 recording_status_endpoint=DEFAULT_STATUS_ENDPOINT, flight_control=None):
        self.endpoint = endpoint
        self.control_endpoint = control_endpoint
        self.blackboard = blackboard
        self.stop = stop
        self.connection_factory = connection_factory
        self.clock = clock
        self.flight_control = flight_control or FlightControl()
        self.connection = None
        self.control_connection = None
        self.reload_requested = Event()
        self.reload_replies = Queue(RELOAD_REPLY_CAPACITY)
        self.last_status_key = None
        self.parameters = None if parameter_config is None else ParameterService(parameter_config)
        self.recording_control_source = recording_control_source
        self.recording = (RecordingBridge(recording_state_endpoint, recording_status_endpoint, clock=clock)
                          if recording_control_source == 'mavlink' else None)

    def open(self):
        self.connection = self.connection_factory(self.endpoint, source_system=SYSTEM_ID,
                                                  source_component=COMPONENT_ID)
        if self.control_endpoint is not None:
            self.control_connection = self.connection_factory(self.control_endpoint,
                                                               source_system=SYSTEM_ID,
                                                               source_component=COMPONENT_ID)

        use_mavlink2(self.connection, SYSTEM_ID, COMPONENT_ID)
        if self.control_connection is not None:
            use_mavlink2(self.control_connection, SYSTEM_ID, COMPONENT_ID)

    def close(self):
        if self.recording is not None:
            self.recording.abort_pending()
            while self.recording.replies and self.control_connection is not None and self.connection is not None:
                requester, result, text, action = self.recording.replies.popleft()
                try:
                    self._recording_ack(requester, result, text, action)
                except Exception:
                    logger.exception('Could not send final recording acknowledgment')
        try:
            if self.recording is not None:
                self.recording.close()
        finally:
            try:
                if self.connection is not None:
                    self.connection.close()
            finally:
                if self.control_connection is not None:
                    self.control_connection.close()

    def take_reload_request(self):
        requested = self.reload_requested.is_set()
        self.reload_requested.clear()
        return requested

    def run(self):
        if self.connection is None:
            raise RuntimeError('MAVLink adapter is not open')
        next_heartbeat = next_telemetry = self.clock()
        while not self.stop.is_set():
            now_s = self.clock()
            feedback = self._fresh_feedback(now_s)
            self._poll_control()
            status = self.blackboard.latest_status()
            if now_s >= next_heartbeat:
                self._send_heartbeat(feedback)
                next_heartbeat = self._next_deadline(next_heartbeat, HEARTBEAT_PERIOD_S, now_s)
            if status is not None:
                current_status_key = status_key(status.value)
                if current_status_key != self.last_status_key:
                    self._send_status(status.value)
                    self.last_status_key = current_status_key
            if now_s >= next_telemetry:
                if feedback is not None:
                    self._send_telemetry(feedback.value)
                self._send_bbox(now_s)
                self._send_flight_state(now_s)
                self._send_recording_state(now_s)
                next_telemetry = self._next_deadline(next_telemetry, TELEMETRY_PERIOD_S, now_s)
            self.stop.wait(SCHEDULER_POLL_S)

    def _poll_control(self):
        while True:
            try:
                command, requester, result, text, action_id = self.flight_control.replies.get_nowait()
            except Empty:
                break
            self.control_connection.mav.command_ack_send(command, result, 0, action_id, *requester)
            self.connection.mav.statustext_send(6 if result == 0 else 3, text.encode()[:50])
        if self.recording is not None:
            self.recording.poll()
            while self.recording.notices:
                value = self.recording.notices.popleft()
                logger.info('Recorder {} path={} error={}', value['state'], value['path'], value['error'])
                self.connection.mav.statustext_send(6, ('Recorder '+value['state']).encode())
            while self.recording.replies:
                requester, result, text, action = self.recording.replies.popleft()
                self._recording_ack(requester, result, text, action)
        if self.control_connection is None:
            return
        try:
            result, text = self.reload_replies.get_nowait()
        except Empty:
            pass
        else:
            self._send_reload_ack(result, text)
        command = self.control_connection.recv_match(
            type=['COMMAND_LONG', 'PARAM_EXT_REQUEST_LIST', 'PARAM_EXT_REQUEST_READ', 'PARAM_EXT_SET'], blocking=False)
        if self.parameters is not None:
            self.parameters.poll(self.control_connection.mav, self.clock())
        if command is None:
            return
        if command.get_type() != 'COMMAND_LONG':
            if (self.parameters is not None and command.target_system in (0, SYSTEM_ID)
                    and command.target_component in (0, COMPONENT_ID)):
                self.parameters.handle(command, self.control_connection.mav)
            return
        if command.command in FLIGHT_COMMANDS:
            # Flight commands require the exact app target, never broadcast.
            if (command.target_system, command.target_component) != (SYSTEM_ID, COMPONENT_ID):
                return
            requester = command.get_srcSystem(), command.get_srcComponent()
            try:
                action, value = parse_command(command.command,
                    [getattr(command, f'param{i}') for i in range(1, 8)])
                if self.parameters is None:
                    raise ValueError('No flight session; detection-only mode')
            except ValueError as error:
                request = (command.command, 'invalid', None, requester, self.clock())
                self.flight_control.reply(request, mavutil.mavlink.MAV_RESULT_DENIED, str(error))
            else:
                self.flight_control.submit(command.command, action, value, requester, self.clock())
            return
        if command.command == RECORDING_CONTROL_COMMAND:
            if command.target_system not in (0, SYSTEM_ID) or command.target_component not in (0, COMPONENT_ID):
                return
            requester = (command.get_srcSystem(), command.get_srcComponent())
            if self.recording_control_source == 'godot':
                self._recording_ack(requester, mavutil.mavlink.MAV_RESULT_DENIED, 'Recording controlled by Godot')
            elif self.recording is None:
                self._recording_ack(requester, mavutil.mavlink.MAV_RESULT_UNSUPPORTED, 'Recording control is not configured')
            elif (not isfinite(command.param1) or min(abs(command.param1), abs(command.param1-1)) > 1e-6
                  or any(not isfinite(getattr(command, f'param{i}')) or abs(getattr(command, f'param{i}')) > 1e-6 for i in range(2, 8))):
                self._recording_ack(requester, mavutil.mavlink.MAV_RESULT_DENIED, 'Invalid recording command parameters')
            else:
                self.recording.request(command.param1 > .5, requester)
            return
        if command.target_system not in (0, SYSTEM_ID) or command.target_component not in (0, COMPONENT_ID):
            return
        if command.command != RELOAD_PROFILE_COMMAND:
            self.control_connection.mav.command_ack_send(command.command, mavutil.mavlink.MAV_RESULT_UNSUPPORTED)
            return
        if not isfinite(command.param1) or abs(command.param1 - RELOAD_PROFILE_PARAMETER) > 1e-6:
            self._send_reload_ack(RELOAD_PROFILE_ACK_DENIED, 'unsupported reload command')
            return
        if self.recording_control_source is not None and self.parameters is None:
            self._send_reload_ack(mavutil.mavlink.MAV_RESULT_DENIED, 'Mission profile is not configured')
            return
        self.reload_requested.set()

    def _recording_ack(self, requester, result, text, action=0):
        self.control_connection.mav.command_ack_send(RECORDING_CONTROL_COMMAND, result, 0, action, *requester)
        logger.info('Recording command result={} {}', result, text)
        self.connection.mav.statustext_send(6 if result == 0 else 3, text.encode('utf-8')[:50])

    def _send_recording_state(self, now_s):
        authority = (RECORDING_AUTH_MAVLINK if self.recording_control_source == 'mavlink' else
                     RECORDING_AUTH_GODOT if self.recording_control_source == 'godot' else RECORDING_AUTH_NONE)
        state = RECORDING_STATES['unknown']
        if self.recording is not None:
            latest = self.recording.latest_status
            if (latest is not None and 0 <= now_s - self.recording.status_received_at <= STATE_TIMEOUT_S
                    and 0 <= now_s - latest['issued_ms'] / 1000 <= STATE_TIMEOUT_S):
                state = RECORDING_STATES[latest['state']]
        stamp = int(now_s * 1000) % 2**32
        for name, value in zip(RECORDING_NAMES, (authority, state)):
            self.connection.mav.named_value_float_send(stamp, name.encode(), float(value))

    def _send_flight_state(self, now_s):
        state = self.flight_control.snapshot()
        fresh = state.valid and 0 <= now_s - state.received_at_s <= STATE_STALE_S
        values = (float(state.armed) if fresh else UNKNOWN_FLOAT, float(fresh),
                  float(state.readiness) if fresh else 0.,
                  state.height_m if fresh and state.height_m is not None else UNKNOWN_FLOAT)
        stamp = int(now_s * 1000) % 2**32
        for name, value in zip(FLIGHT_NAMES, values):
            self.connection.mav.named_value_float_send(stamp, name.encode(), value)

    def reply_reload(self, accepted, message):
        try:
            self.reload_replies.put_nowait((
                RELOAD_PROFILE_ACK_ACCEPTED if accepted else RELOAD_PROFILE_ACK_DENIED, message))
        except Full:
            logger.warning('MAVLink reload acknowledgment queue full')

    def _send_reload_ack(self, result, message):
        self.control_connection.mav.command_ack_send(RELOAD_PROFILE_COMMAND, result)
        self.connection.mav.statustext_send(6, message.encode('utf-8'))

    def _fresh_feedback(self, now_s):
        feedback = self.blackboard.latest_feedback()
        if feedback is None or now_s - feedback.received_at_s > FEEDBACK_STALE_S:
            return None
        return feedback

    def _send_heartbeat(self, feedback):
        armed = feedback is not None and feedback.value['armed']
        running = feedback is not None and feedback.value['state'] == 'running'
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        system_status = (mavutil.mavlink.MAV_STATE_ACTIVE if running
                         else mavutil.mavlink.MAV_STATE_STANDBY)
        self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                           mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                           base_mode, 0, system_status, 3)

    def _send_telemetry(self, feedback):
        source = feedback['source']
        time_us = source['measurement_us']
        roll, pitch, yaw = mavlink_attitude(feedback['orientation_xyzw'])
        self.connection.mav.attitude_send(time_us // 1000 % 2**32, roll, pitch, yaw,
                                          UNKNOWN_FLOAT, UNKNOWN_FLOAT, UNKNOWN_FLOAT)
        altitude = feedback['barometer_altitude_m']
        if altitude is not None:
            self.connection.mav.altitude_send(time_us, UNKNOWN_FLOAT, UNKNOWN_FLOAT, altitude,
                                              UNKNOWN_FLOAT, UNKNOWN_FLOAT, UNKNOWN_FLOAT)
        rangefinder = feedback.get('rangefinder')
        if rangefinder is not None:
            distance_cm = rangefinder['distance_cm'] if rangefinder['valid'] else None
            if distance_cm is None:
                distance_cm = RANGEFINDER_INVALID_CM
            else:
                distance_cm = min(RANGEFINDER_INVALID_CM - 1, max(0, round(distance_cm)))
            self.connection.mav.distance_sensor_send(
                time_us // 1000 % 2**32, RANGEFINDER_MIN_DISTANCE_CM,
                RANGEFINDER_MAX_DISTANCE_CM, distance_cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, 0,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0)

    def _send_bbox(self, now_s):
        vision = self.blackboard.latest_vision()
        bbox = None if vision is None else vision.value[1]
        values = {
            'BBOX_VALID': 1. if bbox is not None else 0.,
            'BBOX_X': UNKNOWN_FLOAT if bbox is None else float(bbox[0]),
            'BBOX_Y': UNKNOWN_FLOAT if bbox is None else float(bbox[1]),
            'BBOX_W': UNKNOWN_FLOAT if bbox is None else float(bbox[2]),
            'BBOX_H': UNKNOWN_FLOAT if bbox is None else float(bbox[3]),
        }
        timestamp_ms = int(now_s * 1000) % 2**32
        for name in BBOX_NAMED_VALUES:
            self.connection.mav.named_value_float_send(timestamp_ms, name.encode('ascii'), values[name])

    def _send_status(self, status):
        reason = status.get('reason', '')
        key = reason if reason in STATUS_MESSAGE_BY_KEY else status.get('phase', '').lower()
        definition = STATUS_MESSAGE_BY_KEY.get(key, STATUS_MESSAGE_BY_KEY['fault'])
        severity = MAV_SEVERITY_ERROR if status.get('phase') == 'FAULT' else definition.severity
        text = f"{status.get('phase', 'UNKNOWN')}: {format_status(status)}"
        simulation_time_s = status.get('simulation_time_s')
        if (type(simulation_time_s) not in (int, float)
                or not isfinite(simulation_time_s) or simulation_time_s < 0):
            simulation_time_s = None
        for chunk in status_text_chunks(definition.index, text,
                simulation_time_s=simulation_time_s, image_sequence=status_image_sequence(status)):
            self.connection.mav.statustext_send(severity, chunk.encode('utf-8'))

    @staticmethod
    def _next_deadline(deadline, period_s, now_s):
        if now_s < deadline:
            return deadline
        return deadline + (int((now_s - deadline) / period_s) + 1) * period_s
