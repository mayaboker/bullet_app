"""Read only target bboxes and confirmed contact from local ZeroMQ publishers."""
import json

import zmq
from bullet_simulation_protocol import validate_endpoint
from bullet_simulation_protocol.scenario import TARGET_CONTACT_TOPIC, validate_target_contact

BBOX_TOPIC = b'tracking/front/bbox'
MAX_PACKET_BYTES = 4096


class TargetIngress:
    def __init__(self, bbox_endpoint: str, contact_endpoint: str):
        validate_endpoint(bbox_endpoint)
        validate_endpoint(contact_endpoint)
        self.context = zmq.Context()
        self.bbox = self.context.socket(zmq.SUB)
        self.contact = self.context.socket(zmq.SUB)
        for socket, endpoint, topic in ((self.bbox, bbox_endpoint, BBOX_TOPIC),
                                        (self.contact, contact_endpoint, TARGET_CONTACT_TOPIC)):
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 16)
            socket.setsockopt(zmq.MAXMSGSIZE, MAX_PACKET_BYTES)
            socket.setsockopt(zmq.SUBSCRIBE, topic)
            socket.connect(endpoint)

    @staticmethod
    def _drain(socket, topic):
        latest = None
        for _ in range(32):
            try:
                parts = socket.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) != 2 or parts[0] != topic or len(parts[1]) > MAX_PACKET_BYTES:
                continue
            try:
                value = json.loads(parts[1].decode(), parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
            except (ValueError, UnicodeError):
                continue
            if isinstance(value, dict):
                latest = value
        return latest

    def poll(self):
        bbox = self._drain(self.bbox, BBOX_TOPIC)
        contact = self._drain(self.contact, TARGET_CONTACT_TOPIC)
        if contact is not None:
            try:
                contact = validate_target_contact(contact)
            except (ValueError, TypeError):
                contact = None
        return bbox, contact

    def close(self):
        self.bbox.close(linger=0)
        self.contact.close(linger=0)
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
