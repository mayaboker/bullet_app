"""Tracker and simulator contact subscriptions stay independent."""
import json

import zmq
from bullet_simulation_protocol.scenario import TARGET_CONTACT_TOPIC

from bullet_app.simulation import SimulationContactSubscriber
from bullet_app.tracker import BBOX_TOPIC, TrackerSubscriber


class Socket:
    def __init__(self, messages):
        self.messages = iter(messages)
        self.closed = False

    def recv_multipart(self, flags):
        assert flags == zmq.NOBLOCK
        try:
            return next(self.messages)
        except StopIteration:
            raise zmq.Again from None

    def close(self, *, linger):
        assert linger == 0
        self.closed = True


def _packet(topic, value):
    return [topic, json.dumps(value).encode()]


def test_tracker_keeps_valid_packets_in_order_and_ignores_malformed():
    subscriber = TrackerSubscriber.__new__(TrackerSubscriber)
    subscriber.socket = Socket([_packet(BBOX_TOPIC, {'sequence': 1}),
                                [b'other', b'{}'], [BBOX_TOPIC, b'{'],
                                _packet(BBOX_TOPIC, {'sequence': 2})])
    assert [packet['sequence'] for packet in subscriber.poll()] == [1, 2]


def test_simulation_returns_latest_valid_contact():
    subscriber = SimulationContactSubscriber.__new__(SimulationContactSubscriber)
    contact = {'schema_version': 1, 'target_id': 'red_truck', 'drone_id': 'drone_1',
               'step_index': 10, 'simulation_time_s': 1.0}
    subscriber.socket = Socket([_packet(TARGET_CONTACT_TOPIC, contact),
                                [TARGET_CONTACT_TOPIC, b'{'],
                                _packet(TARGET_CONTACT_TOPIC, contact)])
    assert subscriber.poll() == contact


def test_each_subscriber_closes_only_its_own_resources():
    class Context:
        terminated = False

        def term(self):
            self.terminated = True

    for subscriber_type in (TrackerSubscriber, SimulationContactSubscriber):
        subscriber = subscriber_type.__new__(subscriber_type)
        subscriber.socket = Socket([])
        subscriber.context = Context()
        subscriber.close()
        assert subscriber.socket.closed and subscriber.context.terminated
