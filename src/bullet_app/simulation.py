"""Receive simulator-only target contact events."""
import json

import zmq
from bullet_simulation_protocol import validate_endpoint
from bullet_simulation_protocol.scenario import TARGET_CONTACT_TOPIC, validate_target_contact

MAX_PACKET_BYTES = 4096
MAX_PACKETS_PER_POLL = 32


class SimulationContactSubscriber:
    """Own the optional ZeroMQ subscription for simulator contact."""

    def __init__(self, endpoint: str):
        validate_endpoint(endpoint)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVHWM, 16)
        self.socket.setsockopt(zmq.MAXMSGSIZE, MAX_PACKET_BYTES)
        self.socket.setsockopt(zmq.SUBSCRIBE, TARGET_CONTACT_TOPIC)
        self.socket.connect(endpoint)

    def poll(self) -> dict | None:
        """Return the latest valid contact event, if received."""
        latest = None
        for _ in range(MAX_PACKETS_PER_POLL):
            try:
                parts = self.socket.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) != 2 or parts[0] != TARGET_CONTACT_TOPIC or len(parts[1]) > MAX_PACKET_BYTES:
                continue
            try:
                value = json.loads(parts[1].decode(), parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
                latest = validate_target_contact(value)
            except (ValueError, UnicodeError, TypeError):
                continue
        return latest

    def close(self) -> None:
        self.socket.close(linger=0)
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
