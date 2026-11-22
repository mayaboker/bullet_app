from types import SimpleNamespace

from bullet_app.cli import DEFAULT_TARGET_PROFILE
from bullet_app.missions.target_hit import TargetMission
from bullet_app.view_link import ViewLink
from bullet_simulation_protocol.view_bbox import MESSAGE_TYPE, decode_bbox
from pymavlink.dialects.v20 import common


def test_app_forwards_accepted_fid_and_no_target():
    calls = []
    link = ViewLink()
    link.connection = SimpleNamespace(mav=SimpleNamespace(v2_extension_send=lambda *args: calls.append(args)))
    now = 10.
    packet = dict(schema_version=3, image_topic='camera/front/image', session='a' * 32,
                  sequence=1, fid=123, capture_ns=int(now * 1e9), width=640, height=480,
                  drone_id='drone_1', bbox={'x': 1, 'y': 2, 'width': 3, 'height': 4})
    mission = TargetMission(DEFAULT_TARGET_PROFILE)
    assert mission.observe_bbox(packet, now)
    assert mission.blackboard.visual.bbox['capture_ns'] == packet['capture_ns']
    assert mission.blackboard.visual.bbox is not packet
    link.publish_bbox(packet)
    assert calls[0][:4] == (0, 255, 190, MESSAGE_TYPE)
    assert decode_bbox(calls[0][4]) == {'fid': 123, 'bbox': packet['bbox']}
    packet['fid'], packet['bbox'] = 124, None
    assert mission.observe_bbox(packet, now)
    link.publish_bbox(packet)
    assert decode_bbox(calls[1][4]) == {'fid': 124, 'bbox': None}
    packet['fid'] = -1
    assert not mission.observe_bbox(packet, now)
    assert mission.blackboard.visual.bbox['fid'] == 124


def test_app_extension_serializes_with_real_pymavlink():
    class Sink:
        data = b''

        def write(self, data):
            self.data += data

    sink = Sink()
    sender = common.MAVLink(sink, srcSystem=1, srcComponent=1)
    link = ViewLink()
    link.connection = SimpleNamespace(mav=sender)
    link.publish_bbox({'fid': 44, 'bbox': {'x': 1, 'y': 2, 'width': 3, 'height': 4}})
    receiver = common.MAVLink(None)
    message = None
    for byte in sink.data:
        parsed = receiver.parse_char(bytes((byte,)))
        if parsed is not None:
            message = parsed
    assert message is not None and message.get_type() == 'V2_EXTENSION'
    assert decode_bbox(message.payload)['fid'] == 44
