import json

import zmq

from bullet_app.ingress import BBOX_TOPIC, TargetIngress


def test_bbox_drain_keeps_all_camera_packets_in_order():
    class Socket:
        def __init__(self):
            self.messages = iter([
                [BBOX_TOPIC, json.dumps({'sequence': 1}).encode()],
                [BBOX_TOPIC, json.dumps({'sequence': 2}).encode()],
            ])

        def recv_multipart(self, flags):
            assert flags == zmq.NOBLOCK
            try:
                return next(self.messages)
            except StopIteration:
                raise zmq.Again from None

    assert [packet['sequence'] for packet in TargetIngress._drain(Socket(), BBOX_TOPIC)] == [1, 2]
