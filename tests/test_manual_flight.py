"""No sockets or motors: deterministic owner-thread manual flight transitions."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from bullet_app.mission import Mission
from bullet_app.flight_control import FlightSession, FlightControl
from bullet_msgs.flight_control import *
from test_mission import mission, feedback, platform, betaflight_config


def ready():
    m = mission()
    m.flight_session.arming_mode = 'manual'
    for step in range(40):
        now = 1000 + step / 100
        m.observe(feedback(step, now, 4.))
        rc = m.tick(now_s=now)
        assert rc['channels'][2:5:2] == [1000, 1000]
    return m, now


def observe(m, now, armed=False):
    m.observe(feedback(m.last_step + 1, now, 4., armed=armed, applied=m.sequence))
    return m.tick(now_s=now)


def test_manual_arm_idle_start_and_height_goal():
    m, now = ready()
    m.manual_command('height', 5.5, now)
    m.manual_command('arm', None, now)
    rc = m.tick(now_s=now)
    assert rc['channels'][4] == 2000 and rc['channels'][2] == 1000
    rc = observe(m, now + .01, True)
    assert m.phase == 'ARMED_IDLE' and rc['channels'][:4] == [1500, 1500, 1000, 1500]
    m.manual_command('start', None, now + .01)
    assert m.goal == 9.5 and m.phase == 'TAKEOFF'
    with pytest.raises(ValueError, match='locked'):
        m.manual_command('height', 8., now + .01)
    assert m.tick(now_s=now + .01)['channels'][2] > 1000
    assert m.status(now)['diagnostics']['takeoff_height_source'] == 'operator'


def test_start_requires_arm_and_fresh_feedback():
    m, now = ready()
    with pytest.raises(ValueError, match='armed idle'):
        m.manual_command('start', None, now)
    with pytest.raises(ValueError, match='Fresh'):
        m.manual_command('arm', None, now + 1)


def test_override_and_disarm_latch_survive_reset_and_replacement():
    m, now = ready()
    m.manual_command('height', 5.5, now)
    m.manual_command('disarm', None, now)
    session = m.flight_session
    for episode in ('new', 'another'):
        m.observe(feedback(0, now + .01, 7., episode=episode))
        rc = m.tick(now_s=now + .01)
        assert rc['channels'][4] == 1000 and m.goal == 12.5
    replacement = Mission(m.config, app_id='app', flight_session=session)
    replacement.configure(platform(m.config))
    replacement.observe(feedback(0, now + .02, 3.))
    assert replacement.tick(now_s=now + .02)['channels'][4] == 1000
    assert replacement.takeoff_height_m == 5.5
    restarted = Mission(m.config)
    assert restarted.takeoff_height_m == 20 and not restarted.flight_session.operator_control


def test_airborne_requires_emergency_and_stop_reset_before_rearm():
    m, now = ready()
    m.manual_command('arm', None, now)
    m.tick(now_s=now)
    observe(m, now + .01, True)
    m.manual_command('start', None, now + .01)
    with pytest.raises(ValueError, match='emergency'):
        m.manual_command('disarm', None, now + .01)
    m.manual_command('emergency', None, now + .01)
    assert not m.tick(now_s=now + .01)['active']
    assert m.tick(now_s=now + .01)['channels'][4] == 1000
    with pytest.raises(ValueError, match='Prearm'):
        m.manual_command('arm', None, now + .01)
    m.observe(feedback(0, now + .02, 4., episode='reset', state='stopped'))
    observe(m, now + .03)
    assert m.flight_session.emergency_identity is None
    assert m.tick(now_s=now + .03)['channels'][4] == 1000


def test_pending_arm_confirms_only_new_observation_and_times_out():
    m, now = ready()
    control = FlightControl()
    control.submit(400, 'arm', None, (255, 190), now)
    control.update(m, now)
    m.tick(now_s=now)
    control.publish(m, now)
    assert control.replies.empty()
    observe(m, now + .01, True)
    control.publish(m, now + .01)
    assert control.replies.get()[2] == 0
    assert control.snapshot().readiness == ARMED_IDLE
    control.submit(400, 'disarm', None, (255, 190), now + .02)
    control.update(m, now + .02)
    control.publish(m, now + 6)
    assert control.replies.get()[2] == 4
    assert control.snapshot().armed is None


def test_disarm_preempts_pending_and_queued_actions():
    m, now = ready()
    c = FlightControl()
    c.submit(400, 'arm', None, (255, 190), now)
    c.update(m, now)
    c.submit(300, 'start', None, (255, 190), now)
    c.submit(400, 'disarm', None, (255, 190), now)
    c.update(m, now)
    assert [c.replies.get()[2] for _ in range(2)] == [6, 6]
    rc = observe(m, now + .01)
    c.publish(m, now + .01)
    assert rc['channels'][4] == 1000 and c.replies.get()[2] == 0


def test_missing_msp_status_never_confirms_disarmed():
    config = betaflight_config()
    m = Mission(config, app_id='app', flight_session=FlightSession('manual'))
    m.configure(dict(platform(config), schema_version=2, backend='betaflight', attitude_model='imu_complementary'))
    m.observe(feedback(0, 1000, 0, armed=True))
    m.observe_estimates((None, None, SimpleNamespace(armed=True, arming_disabled_flags=0, received_at=1000)))
    assert m.flight_feedback_valid(1000)
    m.observe_estimates(None)
    assert m.feedback['armed'] and not m.flight_feedback_valid(1000)


@pytest.mark.parametrize('command,params', [
    (400, [1,21196,0,0,0,0,0]), (400,[2,0,0,0,0,0,0]),
    (300,[1,0,0,0,0,0,0]), (31012,[0,0,0,0,0,0,0]),
    (31012,[100.1,0,0,0,0,0,0]), (31012,[float('nan'),0,0,0,0,0,0]),
    (31012,[5.55,0,0,0,0,0,0]), (400,[1,0,1,0,0,0,0]),
])
def test_strict_command_contract(command, params):
    with pytest.raises(ValueError):
        parse_command(command, params)


def test_udp_flight_height_arm_start_and_emergency_roundtrip():
    """Actual MAVLink 2 UDP serialization; feedback uses a fake internal backend."""
    import socket
    import time
    from threading import Event
    from pymavlink import mavutil
    from bullet_msgs.parameters import use_mavlink2
    from bullet_app.blackboard import RuntimeBlackboard
    from bullet_app.mavlink_adapter import MavlinkAdapter
    from bullet_view.flight_control import FlightClient
    def port():
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.bind(('127.0.0.1',0)); return s.getsockname()[1]
    m, now = ready()
    control = FlightControl()
    telemetry_port, control_port = port(),port()
    adapter=MavlinkAdapter(f'udpout:127.0.0.1:{telemetry_port}',RuntimeBlackboard(),Event(),
        control_endpoint=f'udpin:127.0.0.1:{control_port}',parameter_config=m.config,
        flight_control=control,clock=lambda:now)
    receiver=mavutil.mavlink_connection(f'udpin:127.0.0.1:{telemetry_port}')
    gcs=mavutil.mavlink_connection(f'udpout:127.0.0.1:{control_port}')
    use_mavlink2(gcs,255,190)
    client=FlightClient(lambda snapshot:None)
    adapter.open()
    def pump(done):
        nonlocal now
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            adapter._poll_control()
            control.update(m,now)
            rc=m.tick(now_s=now)
            now+=.01
            observe(m,now,rc['channels'][4]==2000)
            control.publish(m,now)
            adapter._poll_control()
            adapter._send_flight_state(now)
            for connection in (gcs,receiver):
                while (msg:=connection.recv_match(blocking=False)) is not None:
                    if msg.get_type()=='COMMAND_ACK':client.ack(msg,now)
                    elif msg.get_type()=='NAMED_VALUE_FLOAT':client.named(msg,msg.name.rstrip('\0'),now)
            if done():return
            time.sleep(.001)
        pytest.fail(str(client.snapshot()))
    try:
        pump(lambda:client.remote.valid)
        client.request('height',5.5,gcs.mav,now)
        pump(lambda:client.pending is None)
        assert m.takeoff_height_m==5.5 and client.remote.height_m==5.5
        client.request('arm',None,gcs.mav,now)
        pump(lambda:client.pending is None)
        assert m.phase=='ARMED_IDLE' and m.tick(now_s=now)['channels'][:4]==[1500,1500,1000,1500]
        client.request('start',5.5,gcs.mav,now)
        pump(lambda:client.pending is None)
        assert m.goal==9.5 and client.remote.readiness==FLYING
        client.request('emergency',None,gcs.mav,now)
        pump(lambda:client.pending is None)
        assert m.phase=='FAULT' and not client.remote.armed
        assert m.flight_session.operator_control
    finally:
        adapter.close();gcs.close();receiver.close()


def test_scenario_height_without_override_and_captured_inflight_height():
    m=mission()
    for step in range(40):
        now=1000.+step/100
        sample=feedback(step,now,4.)
        sample.update(schema_version=2,takeoff_height_m=7.)
        m.observe(sample)
        rc=m.tick(now_s=now)
    assert m.takeoff_height_m==7. and m.takeoff_height_source=='scenario'
    sample=feedback(40,now+.01,4.,armed=True,applied=rc['sequence'])
    sample.update(schema_version=2,takeoff_height_m=9.)
    m.observe(sample)
    m.tick(now_s=now+.01)
    assert m.phase=='TAKEOFF' and m.goal==11. and m.takeoff_height_m==7.
    sample=feedback(0,now+.02,2.,episode='new')
    sample.update(schema_version=2,takeoff_height_m=3.)
    m.observe(sample)
    assert m.takeoff_height_m==3. and m.goal==5.


def test_adapter_ignores_wrong_target_and_reports_unknown_without_feedback():
    from threading import Event
    from bullet_app.blackboard import RuntimeBlackboard
    from bullet_app.mavlink_adapter import MavlinkAdapter
    from test_mavlink_adapter import Connection
    command=SimpleNamespace(command=400,target_system=2,target_component=1,get_type=lambda:'COMMAND_LONG')
    adapter=MavlinkAdapter('unused',RuntimeBlackboard(),Event())
    adapter.connection=Connection()
    adapter.control_connection=Connection([command])
    adapter._poll_control()
    assert adapter.flight_control.requests.empty() and not adapter.control_connection.mav.acks
    adapter._send_flight_state(1000.)
    names={name.decode():value for _,name,value in adapter.connection.mav.named_values}
    assert names['ARM_VALID']==0 and names['MIS_READY']==0
    from math import isnan
    assert isnan(names['ARM_MODE']) and isnan(names['TAKEOFF_H'])


def test_emergency_stop_then_start_same_episode_requires_explicit_rearm():
    m,now=ready()
    m.manual_command('emergency',None,now)
    assert m.phase=='FAULT'
    m.observe(feedback(m.last_step+1,now+.01,4.,state='stopped'))
    assert m.phase=='READY' and m.flight_session.emergency_identity is None
    observe(m,now+.02)
    assert m.phase=='MANUAL_IDLE' and not m.flight_session.arm_requested
    m.manual_command('arm',None,now+.02)
    assert m.flight_session.arm_requested and m.flight_session.operator_control


def test_emergency_preempts_normal_disarm_in_same_batch():
    m,now=ready();c=FlightControl()
    c.submit(400,'emergency',None,(255,190),now)
    c.submit(400,'disarm',None,(255,190),now)
    c.update(m,now)
    assert m.phase=='FAULT' and m.flight_session.emergency_identity is not None
    assert c.pending[1]=='emergency' and c.replies.get()[2]==6


def test_rc_mission_bit_distinguishes_arm_from_start_and_emergency():
    from bullet_msgs.flight import validate_rc
    m,now=ready()
    m.manual_command('arm',None,now)
    command=m.tick(now_s=now)
    assert validate_rc(command)['schema_version']==2 and not command['mission_started']
    observe(m,now+.01,True)
    command=m.tick(now_s=now+.01)
    assert m.phase=='ARMED_IDLE' and not command['mission_started']
    m.manual_command('start',None,now+.01)
    assert m.tick(now_s=now+.01)['mission_started']
    m.manual_command('emergency',None,now+.01)
    command=m.tick(now_s=now+.01)
    assert not command['active'] and not command['mission_started']
