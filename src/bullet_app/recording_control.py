"""Nonblocking recorder bridge, owned by the MAVLink scheduler thread."""
from collections import deque
import json
from pathlib import Path
import time
from uuid import uuid4

import zmq
from bullet_msgs import validate_endpoint
from bullet_msgs.mavlink_control import RECORDING_ACK_START, RECORDING_ACK_STOP
from bullet_msgs.recording_control import (STATE_TOPIC, STATUS_TOPIC, STATE_INTERVAL_S,
    REQUEST_TIMEOUT_S, MAX_BATCH, MAX_MESSAGE_BYTES, decode, fresh, validate_state, validate_status)


ACK_ACCEPTED = 0
ACK_TEMPORARILY_REJECTED = 1
ACK_FAILED = 4


class RecordingBridge:
    def __init__(self, state_endpoint, status_endpoint, *, clock=time.monotonic):
        for endpoint in (state_endpoint, status_endpoint):
            validate_endpoint(endpoint)
        if state_endpoint == status_endpoint:
            raise ValueError('recording endpoints must differ')
        self.state_endpoint, self.status_endpoint, self.clock = state_endpoint, status_endpoint, clock
        self.context = self.output = self.input = None
        self.controller_id, self.request_id = uuid4().hex, uuid4().hex
        self.session_id = None
        self.sequence, self.status_sequence = 0, 0
        self.recorder_id = None
        self.desired = 'stopped'
        self.next_state = 0.
        self.pending = None
        self.replies = deque(maxlen=MAX_BATCH)
        self.notices = deque(maxlen=MAX_BATCH)
        self.last_notice = None
        self.latest_status = None
        self.status_received_at = float('-inf')

    def _open(self):
        self.context = zmq.Context()
        self.output = self.context.socket(zmq.PUB)
        self.input = self.context.socket(zmq.SUB)
        try:
            self.output.setsockopt(zmq.SNDHWM, MAX_BATCH)
            self.input.setsockopt(zmq.RCVHWM, MAX_BATCH)
            self.input.setsockopt(zmq.MAXMSGSIZE, MAX_MESSAGE_BYTES)
            self.output.setsockopt(zmq.LINGER, 0)
            self.input.setsockopt(zmq.LINGER, 0)
            self.output.bind(self.state_endpoint)
            self.input.setsockopt(zmq.SUBSCRIBE, STATUS_TOPIC)
            self.input.connect(self.status_endpoint)
        except BaseException:
            self.close()
            raise

    def _finish(self, result, text):
        if self.pending is not None:
            for requester in self.pending['requesters']:
                action = RECORDING_ACK_START if self.desired == 'running' else RECORDING_ACK_STOP
                self.replies.append((requester, result, text, action))
        self.pending = None

    def request(self, start, requester):
        desired = 'running' if start else 'stopped'
        if self.pending is not None and desired == self.desired:
            if len(self.pending['requesters']) < MAX_BATCH:
                self.pending['requesters'].add(requester)
            else:
                self.replies.append((requester, ACK_TEMPORARILY_REJECTED, 'Recording command queue full',
                    RECORDING_ACK_START if start else RECORDING_ACK_STOP))
            return
        self._finish(ACK_FAILED, 'Recording request superseded')
        if start and self.desired != 'running':
            self.session_id = uuid4().hex
        self.desired, self.request_id = desired, uuid4().hex
        self.pending = dict(requesters={requester}, deadline=self.clock()+REQUEST_TIMEOUT_S)
        self.next_state = 0.

    def poll(self):
        if self.context is None:
            self._open()
        now = self.clock()
        if now >= self.next_state:
            self.sequence += 1
            value = dict(schema_version=1, controller_id=self.controller_id,
                session_id=self.session_id, request_id=self.request_id, sequence=self.sequence,
                issued_ms=int(now*1000), state=self.desired)
            validate_state(value)
            try:
                self.output.send_multipart([STATE_TOPIC, json.dumps(value).encode()], zmq.NOBLOCK)
            except zmq.Again:
                pass
            self.next_state = now + STATE_INTERVAL_S
        for _ in range(MAX_BATCH):
            try:
                parts = self.input.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) != 2 or parts[0] != STATUS_TOPIC:
                continue
            try:
                value = decode(parts[1], validate_status)
            except (ValueError, TypeError, UnicodeError, RecursionError):
                continue
            if not fresh(value, now) or value['controller_id'] != self.controller_id:
                continue
            if self.recorder_id != value['recorder_id']:
                self.recorder_id, self.status_sequence = value['recorder_id'], 0
            if value['sequence'] <= self.status_sequence:
                continue
            self.status_sequence = value['sequence']
            self.latest_status, self.status_received_at = value, now
            notice = (value['state'], value['path'], value['error'])
            if notice != self.last_notice:
                self.notices.append(value)
                self.last_notice = notice
            if (self.pending is None and value['request_id'] == self.request_id
                    and value['state'] in ('idle', 'error') and self.desired == 'running'):
                # Recorder timeout/error closed this session; the next Start needs a new ID.
                self.desired, self.next_state = 'stopped', 0.
            if self.pending is None or value['request_id'] != self.request_id:
                continue
            if value['state'] == 'error':
                self._finish(ACK_FAILED, value['error'] or 'Recorder failed')
                self.desired, self.next_state = 'stopped', 0.
            elif self.desired == 'running' and value['state'] == 'recording' and value['session_id'] == self.session_id:
                self._finish(ACK_ACCEPTED, 'Recording started: '+Path(value['path'] or '').name)
            elif self.desired == 'stopped' and value['state'] == 'idle':
                self._finish(ACK_ACCEPTED, 'Recording stopped; file finalized')
        if self.pending is not None and now >= self.pending['deadline']:
            self._finish(ACK_FAILED, 'Recorder confirmation timed out')
            self.desired, self.next_state = 'stopped', 0.

    def close(self):
        if self.input is not None:
            self.input.close(0)
        if self.output is not None:
            self.output.close(0)
        if self.context is not None:
            self.context.term()
        self.context = self.input = self.output = None

    def abort_pending(self):
        self._finish(ACK_FAILED, 'Application stopped before recorder confirmation')
        self.desired = 'stopped'
