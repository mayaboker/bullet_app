from threading import Event
from types import SimpleNamespace
import socket
import json
import zmq

from pymavlink import mavutil

from bullet_app.blackboard import RuntimeBlackboard
from bullet_app.mavlink_adapter import MavlinkAdapter
from bullet_app.recording_control import RecordingBridge
from bullet_msgs.mavlink_control import RECORDING_CONTROL_COMMAND


def endpoint():
    with socket.socket() as s:
        s.bind(('127.0.0.1',0));return f'tcp://127.0.0.1:{s.getsockname()[1]}'


def test_confirmation_timeout_stops_late_recording_request():
    clock=[100.]
    bridge=RecordingBridge(endpoint(),endpoint(),clock=lambda:clock[0])
    try:
        bridge.request(True,(255,190));bridge.poll()
        assert not bridge.replies
        clock[0]+=5.1;bridge.poll()
        assert bridge.replies.popleft()[1]==mavutil.mavlink.MAV_RESULT_FAILED
        assert bridge.desired=='stopped'
        bridge.request(True,(255,190));bridge.poll()
        assert bridge.pending is not None
    finally:bridge.close()


def test_shutdown_explicitly_fails_pending_recording_request():
    bridge=RecordingBridge(endpoint(),endpoint())
    bridge.request(True,(255,190))
    bridge.abort_pending()
    assert bridge.replies.popleft()[1]==mavutil.mavlink.MAV_RESULT_FAILED
    assert bridge.pending is None and bridge.desired=='stopped'


def test_recorder_timeout_allows_next_start_to_use_new_session():
    bridge=RecordingBridge(endpoint(),endpoint(),clock=lambda:100.)
    bridge.desired='running';bridge.session_id='old-session'
    value=dict(schema_version=1,recorder_id='recorder',controller_id=bridge.controller_id,
        session_id='old-session',request_id=bridge.request_id,sequence=1,issued_ms=100000,
        state='idle',path='/tmp/closed.mcap',error=None)
    class Input:
        def recv_multipart(self,flags):
            if not hasattr(self,'sent'):
                self.sent=True;return [b'recording/status',json.dumps(value).encode()]
            raise zmq.Again()
    bridge.context=object();bridge.input=Input()
    bridge.output=SimpleNamespace(send_multipart=lambda *args:None)
    bridge.poll()
    assert bridge.desired=='stopped'
    assert bridge.latest_status==value and bridge.status_received_at==100.
    bridge.request(True,(255,190))
    assert bridge.session_id!='old-session'


def test_godot_authority_rejects_mavlink_and_wrong_targets():
    acks=[]
    packet=SimpleNamespace(command=RECORDING_CONTROL_COMMAND,target_system=1,target_component=1,
        get_type=lambda:'COMMAND_LONG',get_srcSystem=lambda:255,get_srcComponent=lambda:190)
    connection=SimpleNamespace(mav=SimpleNamespace(command_ack_send=lambda *v:acks.append(v),
        statustext_send=lambda *v:None),recv_match=lambda **kw:packet)
    adapter=MavlinkAdapter('unused',RuntimeBlackboard(),Event(),recording_control_source='godot')
    adapter.connection=adapter.control_connection=connection
    adapter._poll_control()
    assert acks[0][:2]==(RECORDING_CONTROL_COMMAND,mavutil.mavlink.MAV_RESULT_DENIED)
    packet.target_component=200
    adapter._poll_control();assert len(acks)==1


def test_detection_only_reload_is_denied_instead_of_left_pending():
    acks=[]
    packet=SimpleNamespace(command=31010,target_system=1,target_component=1,param1=1.,
        get_type=lambda:'COMMAND_LONG')
    connection=SimpleNamespace(mav=SimpleNamespace(command_ack_send=lambda *v:acks.append(v),
        statustext_send=lambda *v:None),recv_match=lambda **kw:packet)
    adapter=MavlinkAdapter('unused',RuntimeBlackboard(),Event(),recording_control_source='godot')
    adapter.connection=adapter.control_connection=connection
    adapter._poll_control()
    assert acks==[(31010,mavutil.mavlink.MAV_RESULT_DENIED)]
    assert not adapter.take_reload_request()
    packet.command=400
    packet.get_srcSystem=lambda:255
    packet.get_srcComponent=lambda:190
    for i in range(2,8): setattr(packet, f'param{i}', 0.)
    adapter._poll_control()
    packet.command=999  # drain asynchronous flight denial, then unknown command
    adapter._poll_control()
    assert acks[-2]==(400,mavutil.mavlink.MAV_RESULT_DENIED,0,0,255,190)
    assert acks[-1]==(999,mavutil.mavlink.MAV_RESULT_UNSUPPORTED)


def test_recording_telemetry_is_actual_state_and_expires():
    from test_mavlink_adapter import Connection
    from bullet_msgs.mavlink_control import RECORDING_ACK_START, RECORDING_ACK_STOP
    adapter=MavlinkAdapter('unused',RuntimeBlackboard(),Event(),recording_control_source='mavlink')
    adapter.connection=Connection()
    adapter._send_recording_state(100.)
    assert [item[2] for item in adapter.connection.mav.named_values]==[1.,-1.]
    adapter.recording.latest_status=dict(state='recording',issued_ms=100000)
    adapter.recording.status_received_at=100.
    adapter.recording.desired='stopped'  # desired Stop must not look like observed idle
    adapter._send_recording_state(102.)
    assert adapter.connection.mav.named_values[-1][2]==1.
    adapter._send_recording_state(104.)
    assert adapter.connection.mav.named_values[-1][2]==-1.
    bridge=adapter.recording
    bridge.request(True,(255,190))
    bridge.request(False,(255,190))
    assert bridge.replies.popleft()[3]==RECORDING_ACK_START
    bridge._finish(0,'Finalized')
    assert bridge.replies.popleft()[3]==RECORDING_ACK_STOP
