from dataclasses import asdict
import json
from pathlib import Path
from threading import Event
from types import SimpleNamespace
import socket
import time

import pytest
from jsonschema import validate
from loguru import logger
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as wire
from bullet_msgs import schema_bytes
from bullet_msgs.parameters import (BY_PATH, REVISION_ID, profile_values, use_mavlink2)
from bullet_app.parameters import (ParameterService, ParameterRequest, apply_request,
    ACK_ACCEPTED, ACK_FAILED)
from bullet_app.blackboard import RuntimeBlackboard, OUTPUT_EVENT_QUEUE_CAPACITY
from bullet_app.mavlink_adapter import MavlinkAdapter
from bullet_app.mission import Mission
from los_tracking import NavigatorConfig, State
from bullet_view.parameters import ParameterClient

PROFILE=Path(__file__).resolve().parents[3]/'config/betaflight_los_heavy_quad.yaml'
SOURCE=dict(physics_id='physics',episode_id='episode',stream_id='stream',step_index=1,
            simulation_time_s=1.,measurement_us=1_000_000)


def setup_runtime():
    config=NavigatorConfig.from_yaml(PROFILE)
    mission=Mission(config)
    mission.phase='LOS_APPROACH'
    mission.navigator.state=State.TRACK
    mission.feedback=dict(state='running',source=SOURCE)
    service=ParameterService(config)
    runtime=SimpleNamespace(config=config,mavlink=SimpleNamespace(parameters=service),
                            blackboard=RuntimeBlackboard(),hover_check=False)
    return runtime,mission,service


def test_runtime_logs_and_records_before_ack_and_duplicate_is_idempotent():
    runtime,mission,service=setup_runtime()
    p=BY_PATH['tuning.azimuth_kp']
    request=ParameterRequest(p.id,1.5,(255,190))
    logs=[]
    sink=logger.add(lambda message:logs.append(str(message)))
    try:
        mission,_=apply_request(runtime,mission,request,None,None,10.,{})
        assert runtime.config.tuning.azimuth_kp==1.5
        assert mission.navigator.config==mission.hold.config==mission.config==runtime.config
        event=runtime.blackboard.take_publication_inputs().parameter_events[0].value
        validate(event,json.loads(schema_bytes('analysis_parameters.json')))
        assert event['old_value']==2.2 and event['effective_value']==1.5
        assert event['config_revision']==1
        assert 'Parameter applied' in logs[0]
        assert service.replies.get_nowait()[1]==ACK_ACCEPTED
        mission,_=apply_request(runtime,mission,request,None,None,11.,{})
        assert service.snapshot()[REVISION_ID]==1
        assert not runtime.blackboard.take_publication_inputs().parameter_events
        assert len(logs)==1
    finally:logger.remove(sink)


@pytest.mark.parametrize('path,value,phase,reason',[
    ('tuning.azimuth_kp',-1.,'LOS_APPROACH','nonnegative'),
    ('tuning.roll_limit_rad',.3,'LOS_APPROACH','stopped'),
    ('tuning.azimuth_kp',1.,'LOS_COMMIT','COMMIT'),
])
def test_rejected_write_keeps_configuration(path,value,phase,reason):
    runtime,mission,service=setup_runtime()
    old=runtime.config
    mission.phase=phase
    mission,_=apply_request(runtime,mission,ParameterRequest(BY_PATH[path].id,value,(255,190)),None,None,10.,{})
    assert runtime.config==old
    assert service.replies.get_nowait()[1]==ACK_FAILED
    event=runtime.blackboard.take_publication_inputs().parameter_events[0].value
    assert event['result']=='rejected' and reason in event['reason']
    assert service.snapshot()[REVISION_ID]==0


def test_stopped_edit_rebuilds_but_running_edit_does_not():
    runtime,mission,service=setup_runtime()
    mission.phase='READY';mission.navigator.state=State.DISABLED
    mission.feedback['state']='stopped'
    replacement,_=apply_request(runtime,mission,ParameterRequest(BY_PATH['tuning.roll_limit_rad'].id,.3,(255,190)),None,None,10.,{})
    assert replacement is not mission
    assert replacement.config.tuning.roll_limit_rad==.3
    assert service.replies.get_nowait()[1]==ACK_ACCEPTED


def test_parameter_event_queue_is_bounded_and_reports_drops():
    board=RuntimeBlackboard()
    for i in range(OUTPUT_EVENT_QUEUE_CAPACITY+2):board.enqueue_parameters({'sequence':i},float(i))
    inputs=board.take_publication_inputs()
    assert len(inputs.parameter_events)==OUTPUT_EVENT_QUEUE_CAPACITY
    assert inputs.parameter_events[0].value['sequence']==2
    assert inputs.health.dropped_events['parameters']==2


def free_port():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1',0));return sock.getsockname()[1]


def test_udp_sync_live_edit_rejection_and_readback():
    runtime,mission,_=setup_runtime()
    port=free_port()
    adapter=MavlinkAdapter(f'udpout:127.0.0.1:{free_port()}',runtime.blackboard,Event(),
        control_endpoint=f'udpin:127.0.0.1:{port}',parameter_config=runtime.config)
    runtime.mavlink=adapter
    adapter.open()
    gcs=mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}')
    use_mavlink2(gcs,255,190)
    client=ParameterClient()
    client.target=(1,1);client.online=True;client.last_heartbeat=time.monotonic()
    client.refresh(gcs.mav,time.monotonic())
    def pump(until, timeout=10.):
        nonlocal mission
        deadline=time.monotonic()+timeout
        while not until() and time.monotonic()<deadline:
            now=time.monotonic()
            client.last_heartbeat=now
            adapter._poll_control()
            for request in adapter.parameters.take_requests():
                mission,_=apply_request(runtime,mission,request,None,None,now,{})
            message=gcs.recv_match(type=['PARAM_EXT_VALUE','PARAM_EXT_ACK'],blocking=False)
            if message is not None:client.handle(message,now,gcs.mav)
            client.poll(gcs.mav,now)
            time.sleep(.001)
        assert until(),client.snapshot()
    try:
        pump(lambda:client.complete)
        assert len(client.values)==len(adapter.parameters.snapshot())
        client.set(gcs.mav,'LOS_AZ_KP',1.5,time.monotonic())
        pump(lambda:client.complete and client.pending is None)
        assert client.values['LOS_AZ_KP']==1.5
        assert mission.navigator.config.tuning.azimuth_kp==1.5
        client.set(gcs.mav,'LOS_AZ_KP',-1.,time.monotonic())
        pump(lambda:client.complete and client.pending is None)
        assert client.values['LOS_AZ_KP']==1.5
        assert client.statuses['LOS_AZ_KP']=='Rejected'
        assert adapter.parameters.revision==1
    finally:
        gcs.close();adapter.close()


def test_parameter_events_and_effective_profile_reach_mcap(tmp_path):
    from dataclasses import replace
    from mcap.reader import make_reader
    from mcap_bagger.config import Config
    from mcap_bagger.recorder import Recording
    from bullet_app.runtime_publisher import RuntimePublisher
    runtime,mission,service=setup_runtime()
    apply_request(runtime,mission,ParameterRequest('LOS_AZ_KP',1.5,(255,190)),None,None,10.,{})
    inputs=runtime.blackboard.take_publication_inputs()
    config=Config.load(PROFILE.parents[1]/'projects/bullet_render/configs/recorder.yaml')
    keys={'analysis/app/parameters','analysis/app/config'}
    topics=tuple(replace(t,endpoint=f'tcp://127.0.0.1:{free_port()}') for t in config.topics if t.key in keys)
    assert len(topics)==2
    config=replace(config,topics=topics,output_dir=tmp_path)
    recording=Recording(config)
    packets=[]
    socket=SimpleNamespace(send_multipart=lambda parts,**kwargs:packets.append(parts))
    publisher=RuntimePublisher({},'',runtime.blackboard,Event())
    publisher._publish_events(SimpleNamespace(socket=socket),inputs)
    publisher._publish_latest(socket,socket,inputs,10.,0.,0.)
    recording.start()
    try:
        by_key={t.key:t for t in topics}
        for topic,payload in packets:
            if topic.decode() in keys:recording.receive(by_key[topic.decode()],payload)
    finally:recording.close()
    assert recording.error is None
    with recording.path.open('rb') as stream:
        messages={c.topic:json.loads(m.data) for _,c,m in make_reader(stream).iter_messages() if c.topic in keys}
    assert messages['analysis/app/parameters']['effective_value']==1.5
    assert messages['analysis/app/config']['settings']['tuning']['azimuth_kp']==1.5


@pytest.mark.parametrize('path,value',[('tuning.azimuth_kp',1.5),('tuning.roll_limit_rad',.3)])
def test_completed_committed_mission_can_be_edited_while_stopped(path,value):
    runtime,mission,service=setup_runtime()
    mission.phase='COMPLETED';mission.navigator.state=State.COMMIT
    mission.feedback['state']='stopped'
    replacement,_=apply_request(runtime,mission,ParameterRequest(BY_PATH[path].id,value,(255,190)),None,None,10.,{})
    assert replacement is not mission
    assert service.replies.get_nowait()[1]==ACK_ACCEPTED
    assert replacement.navigator.state==State.DISABLED


def test_source_profile_paths_are_recorded_and_survive_runtime_edits(tmp_path):
    from bullet_app.runtime import MissionRuntime
    from bullet_app.config_snapshot import configuration_snapshot
    runtime,mission,service=setup_runtime()
    profile=tmp_path/'mission-profile.yaml';transport=tmp_path/'transport.json'
    configured=MissionRuntime({},runtime.config,bbox_endpoint='tcp://127.0.0.1:5558',
                               mission_config=profile,transport_config=transport)
    runtime.config_files=configured.config_files
    assert runtime.config_files==dict(mission_profile=str(profile),transport_config=str(transport))
    periodic=configuration_snapshot(runtime.config,SOURCE,{},runtime.config_files)
    validate(periodic,json.loads(schema_bytes('analysis_config.json')))
    assert 'files' not in periodic['settings']
    mission,_=apply_request(runtime,mission,ParameterRequest('LOS_AZ_KP',1.5,(255,190)),None,None,10.,{})
    changed=runtime.blackboard.take_publication_inputs().config_snapshot.value
    assert changed['files']==periodic['files']
    assert changed['settings']['tuning']['azimuth_kp']==1.5
