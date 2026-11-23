"""MAVLink restart gate and lifecycle status while the flight link is absent."""
from types import SimpleNamespace

from pymavlink import mavutil

from bullet_app.blackboard import Blackboard, StepContext
from bullet_app.view_link import APP_STATES, RESTART_COMMAND, ViewLink


def _message(command=RESTART_COMMAND, source=(255, 190), target=(1, 1), params=None):
    fields = {f'param{i}': value for i, value in enumerate(params or [0.] * 7, 1)}
    return SimpleNamespace(command=command, target_system=target[0], target_component=target[1],
                           get_type=lambda: 'COMMAND_LONG', get_srcSystem=lambda: source[0],
                           get_srcComponent=lambda: source[1], **fields)


def test_restart_command_validates_source_target_heartbeat_and_parameters():
    messages = [_message(source=(2, 2)), _message(target=(2, 2)),
                _message(params=[1.] + [0.] * 6), _message()]
    acks, calls = [], []
    mav = SimpleNamespace(command_ack_send=lambda *args: acks.append(args),
                          statustext_send=lambda *_: None)
    link = ViewLink()
    link.connection = SimpleNamespace(mav=mav, recv_match=lambda **_: messages.pop(0) if messages else None)
    link.last_view_at = 10.
    link.poll(None, StepContext(Blackboard(), 10.), lambda: calls.append('restart'))
    assert calls == ['restart']
    assert [ack[1] for ack in acks] == [mavutil.mavlink.MAV_RESULT_DENIED,
                                         mavutil.mavlink.MAV_RESULT_IN_PROGRESS]
    assert all(ack[0] == RESTART_COMMAND for ack in acks)


def test_restart_reports_failure_and_offline_state():
    values, acks = [], []
    mav = SimpleNamespace(named_value_float_send=lambda *args: values.append(args),
                          heartbeat_send=lambda *_: None,
                          command_ack_send=lambda *args: acks.append(args),
                          statustext_send=lambda *_: None)
    link = ViewLink()
    link.connection = SimpleNamespace(mav=mav)
    link.app_state = 'restart_failed'
    link.publish_app_status(10., session_ready=False)
    link.finish_restart(False, 'No SITL')
    assert (b'APP_STATE', float(APP_STATES['restart_failed'])) in [x[1:] for x in values]
    assert acks[-1][1] == mavutil.mavlink.MAV_RESULT_FAILED
