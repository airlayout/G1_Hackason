"""Lazy DDS access. Constructing ReadOnlyState never creates a command publisher."""
import os
import ipaddress
import json
import re
import socket
import sys
import time
import threading
from collections import Counter, deque
import numpy as np
from common import ROOT, ARMS, ARM_INDICES, validate_arm_pose

# Extract both arms, but the first physical trial commands ONLY the right arm.
ACTIVE = [i for i,j in enumerate(ARMS) if j['name'].startswith('right_')]

ARM_SDK_TOPIC = 'rt/arm_sdk'
ARM_SDK_INPUT_TOPIC = 'rt/armsdk'
ARM_ACTION_STATE_TOPIC = 'rt/arm/action/state'
EXPECTED_ARM_HOST = 'Unitree'
EXPECTED_ARM_IP = '192.168.123.161'
EXPECTED_ARM_PROCESS = 'python3'
ARM_ACTION_STALE_S = 0.5
LOWSTATE_FRESHNESS_S = 0.15
LOWSTATE_BACKENDS = frozenset({"unitree", "cyclonedds"})
EXPECTED_ARM_PUBLICATIONS = {
    ARM_SDK_TOPIC, 'rt/api/arm/response', 'rt/api/sport/request', ARM_ACTION_STATE_TOPIC,
}
EXPECTED_ARM_SUBSCRIPTIONS = {
    ARM_SDK_INPUT_TOPIC, 'rt/api/arm/request', 'rt/api/sport/response',
    'rt/lf/bmsstate', 'rt/lowstate', 'rt/sportmodestate',
}

def _qos_properties(qos):
    return {str(p.key):str(p.value) for p in qos
            if hasattr(p,'key') and hasattr(p,'value')}

def _advertised_ips(properties):
    result=[]
    for locator in properties.get('__NetworkAddresses','').split(','):
        match=re.match(r'\s*udp/([^:@]+):\d+(?:@\d+)?\s*$',locator)
        if match and not match.group(1).startswith('239.'):
            result.append(match.group(1))
    return sorted(set(result))

def decode_arm_action_state(raw):
    """Decode only the observed official String_ JSON shape; never guess fields."""
    try:data=json.loads(raw)
    except (TypeError,json.JSONDecodeError):return {'status':'UNKNOWN','raw':raw,'reason':'not JSON'}
    if not isinstance(data,dict) or set(data)!={'holding','id','name'}:
        return {'status':'UNKNOWN','raw':raw,'decoded':data,'reason':'unrecognized JSON fields'}
    if not isinstance(data['holding'],bool) or not isinstance(data['id'],int) or not isinstance(data['name'],str):
        return {'status':'UNKNOWN','raw':raw,'decoded':data,'reason':'unrecognized JSON field types'}
    if data=={'holding':False,'id':0,'name':''}:
        status='IDLE'
    elif data['holding'] or data['id']!=0 or data['name']:
        status='ACTIVE'
    else:
        status='UNKNOWN'
    return {'status':status,'raw':raw,'decoded':data,
            'reason':'exact observed holding/id/name schema'}


def arm_action_ownership_reasons(status,allow_unknown=False):
    if status=='IDLE':return []
    if status=='ACTIVE':return ['fresh Arm Action is explicitly non-IDLE']
    if allow_unknown:return []
    return ['Arm Action is not confirmed idle']


def load_sdk():
    bundled_cyclonedds=ROOT/'external/cyclonedds/lib/libddsc.so'
    if bundled_cyclonedds.exists():
        os.environ['CYCLONEDDS_HOME']=str(bundled_cyclonedds.parents[1])
    sys.path.insert(0,str(ROOT/'external'))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    return ChannelFactoryInitialize,ChannelSubscriber,ChannelPublisher,LowState_,LowCmd_,unitree_hg_msg_dds__LowCmd_,CRC

class ReadOnlyState:
    def __init__(self,interface,discovery_peers=(),expected_arm_pid=None,
                 arm_action_stale_s=ARM_ACTION_STALE_S,lowstate_backend="unitree"):
        if not interface or interface not in os.listdir('/sys/class/net') or interface=='lo':
            raise ValueError('Select an existing physical G1 network interface explicitly')
        peers=[str(ipaddress.IPv4Address(value)) for value in discovery_peers]
        if expected_arm_pid is None or not str(expected_arm_pid).isdigit():
            raise ValueError('Expected Arm PID must come from the current identity probe')
        self.expected_arm_pid=str(expected_arm_pid)
        self.arm_action_stale_s=float(arm_action_stale_s)
        if not np.isfinite(self.arm_action_stale_s) or self.arm_action_stale_s<=0:
            raise ValueError('Arm Action freshness limit must be finite and positive')
        if lowstate_backend not in LOWSTATE_BACKENDS:
            raise ValueError('LowState backend must be unitree or cyclonedds')
        self.lowstate_backend=lowstate_backend
        self.sdk=load_sdk();init,sub,_,state,cmd,_,_=self.sdk
        # Official sample enables config tracing to /tmp. Disable it locally:
        # older CycloneDDS builds can abort in their tracing formatter on this PC.
        from unitree_sdk2py.core import channel
        peer_xml=''.join(f'<Peer Address="{value}"/>' for value in peers)
        discovery=f'<Discovery><Peers>{peer_xml}</Peers></Discovery>' if peers else ''
        config='''<CycloneDDS><Domain Id="0"><General><Interfaces>
          <NetworkInterface name="$__IF_NAME__$"/></Interfaces></General>
          $__DISCOVERY__$
          <Tracing><Verbosity>none</Verbosity></Tracing></Domain></CycloneDDS>'''.replace(
              '$__DISCOVERY__$',discovery)
        channel.ChannelConfigHasInterface=config
        os.environ['CYCLONEDDS_URI']=config.replace('$__IF_NAME__$',interface)
        self.network_interface=interface;self.discovery_peers=peers
        init(0,interface)
        # Separate receive-only discovery participant. Detect even idle writers,
        # and refuse multiple LowState sources on the selected network.
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.builtin import (BuiltinDataReader,BuiltinTopicDcpsParticipant,
                                       BuiltinTopicDcpsPublication,BuiltinTopicDcpsSubscription)
        from cyclonedds.core import (Listener,ReadCondition,SampleState,ViewState,
                                    InstanceState)
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        self.discovery_participant=DomainParticipant(0)
        self.discovery=BuiltinDataReader(self.discovery_participant,BuiltinTopicDcpsPublication)
        self.alive=ReadCondition(self.discovery,SampleState.Any|ViewState.Any|InstanceState.Alive)
        self.participant_discovery=BuiltinDataReader(
            self.discovery_participant,BuiltinTopicDcpsParticipant)
        self.participant_alive=ReadCondition(
            self.participant_discovery,SampleState.Any|ViewState.Any|InstanceState.Alive)
        self.subscription_discovery=BuiltinDataReader(
            self.discovery_participant,BuiltinTopicDcpsSubscription)
        self.subscription_alive=ReadCondition(
            self.subscription_discovery,SampleState.Any|ViewState.Any|InstanceState.Alive)
        self.lock=threading.Lock();self.history=deque(maxlen=1000);self.latest=None
        # Keep numeric copies of the high-rate LowState stream.  The 50 Hz arm
        # loop consumes this read-only history for velocity characterization;
        # it does not derive a command from these samples.
        self.motion_history=deque(maxlen=5000);self.motion_sequence=0
        self.arm_packets=0;self.last_tick=None;self.lowstate_reader_error=None
        self.lowstate_closing=False
        self.activity={topic:{'count':0,'by_handle':Counter(),'first_monotonic':{},
                              'last_monotonic':{},'first_source_timestamp_ns':{},
                              'last_source_timestamp_ns':{}}
                       for topic in (ARM_SDK_TOPIC,ARM_SDK_INPUT_TOPIC,ARM_ACTION_STATE_TOPIC)}
        self.action_messages=deque(maxlen=1000)
        self.arm_command_messages=deque(maxlen=2000)
        self._start_lowstate_reader(sub,state,DataReader,Topic,Listener)
        # These are direct receive-only readers on the discovery participant so
        # sample_info.publication_handle maps to the discovered endpoint handle.
        self.activity_topics=[];self.activity_listeners=[];self.activity_readers=[]
        for topic_name,topic_type in ((ARM_SDK_TOPIC,cmd),(ARM_SDK_INPUT_TOPIC,cmd),
                                      (ARM_ACTION_STATE_TOPIC,String_)):
            topic=Topic(self.discovery_participant,topic_name,topic_type)
            listener=Listener(on_data_available=lambda data_reader,n=topic_name:
                              self._activity_available(n,data_reader))
            data_reader=DataReader(self.discovery_participant,topic,None,listener)
            self.activity_topics.append(topic);self.activity_listeners.append(listener)
            self.activity_readers.append(data_reader)

    def _start_lowstate_reader(self,subscriber,state_type,data_reader,topic,listener):
        if self.lowstate_backend=='unitree':
            self.reader=subscriber('rt/lowstate',state_type)
            self.reader.Init(self._receive,10)
            return
        self.lowstate_topic=topic(self.discovery_participant,'rt/lowstate',state_type)
        self.lowstate_listener=listener(on_data_available=self._direct_lowstate_available)
        self.reader=data_reader(
            self.discovery_participant,self.lowstate_topic,None,self.lowstate_listener)

    def _activity_available(self,topic,data_reader):
        try:samples=data_reader.take(1000)
        except Exception:return
        now=time.monotonic()
        with self.lock:
            activity=self.activity[topic]
            for msg in samples:
                info=getattr(msg,'sample_info',None)
                if info is None or not info.valid_data:continue
                handle=str(int(info.publication_handle))
                activity['count']+=1;activity['by_handle'][handle]+=1
                activity['first_monotonic'].setdefault(handle,now)
                activity['last_monotonic'][handle]=now
                activity['first_source_timestamp_ns'].setdefault(
                    handle,int(info.source_timestamp))
                activity['last_source_timestamp_ns'][handle]=int(info.source_timestamp)
                if topic==ARM_SDK_TOPIC:self.arm_packets+=1
                if topic==ARM_SDK_TOPIC:
                    try:
                        self.arm_command_messages.append({
                            'received_monotonic':now,
                            'publication_handle':handle,
                            'weight':float(msg.motor_cmd[29].q),
                            'q':tuple(float(msg.motor_cmd[i].q) for i in range(29)),
                        })
                    except (AttributeError,IndexError,TypeError,ValueError):
                        self.arm_command_messages.append({
                            'received_monotonic':now,
                            'publication_handle':handle,
                            'decode_error':True,
                        })
                if topic==ARM_ACTION_STATE_TOPIC:
                    self.action_messages.append({'received_monotonic':now,
                                                 'source_timestamp_ns':int(info.source_timestamp),
                                                 'publication_handle':handle,'data':msg.data})

    def _direct_lowstate_available(self,data_reader):
        if data_reader is None:
            with self.lock:closing=self.lowstate_closing
            if closing:return
            detail="AttributeError: DataReader is None outside cleanup"
            with self.lock:self.lowstate_reader_error=detail
            print(f'[Direct LowState] take failed: {detail}',file=sys.stderr,flush=True)
            return
        try:samples=data_reader.take(4096)
        except BaseException as exc:
            detail=f'{type(exc).__name__}: {exc!r}'
            with self.lock:self.lowstate_reader_error=detail
            print(f'[Direct LowState] take failed: {detail}',file=sys.stderr,flush=True)
            return
        now=time.monotonic()
        for msg in samples or ():
            info=getattr(msg,'sample_info',None)
            if info is not None and info.valid_data:self._receive(msg,now)

    def _receive(self,msg,received=None):
        with self.lock:
            # A stream repeating a frozen tick is not fresh feedback.
            if msg.tick==self.last_tick:return
            self.last_tick=msg.tick
            now=time.monotonic() if received is None else float(received)
            self.latest=(now,msg);self.history.append((now,msg))
            self.motion_sequence+=1
            self.motion_history.append({
                'sequence':self.motion_sequence,'monotonic_s':now,'tick':int(msg.tick),
                'q':tuple(float(m.q) for m in msg.motor_state[:29]),
                'dq':tuple(float(m.dq) for m in msg.motor_state[:29]),
            })

    def motion_history_snapshot(self,after_sequence=None,lookback_s=.11):
        """Copy new motion samples plus enough history for 100 ms velocity windows."""
        with self.lock:
            rows=list(self.motion_history)
        if after_sequence is None or not rows:
            return [dict(row) for row in rows]
        first=next((i for i,row in enumerate(rows)
                    if int(row['sequence'])>int(after_sequence)),len(rows))
        if first==len(rows):
            return []
        cutoff=float(rows[first]['monotonic_s'])-float(lookback_s)
        start=first
        while start>0 and float(rows[start-1]['monotonic_s'])>cutoff:
            start-=1
        if start>0:start-=1
        return [dict(row) for row in rows[start:]]

    def get(self,max_age=LOWSTATE_FRESHNESS_S):
        with self.lock:latest=self.latest;reader_error=self.lowstate_reader_error
        if reader_error is not None:
            raise RuntimeError(f'LowState reader failed; refuse motion: {reader_error}')
        if latest is None or time.monotonic()-latest[0]>max_age:
            raise RuntimeError('LowState absent/stale; refuse motion')
        return latest[1]

    def wait(self,seconds=3):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:time.sleep(.02)
        return self.get()

    def require_stationary(self):
        cutoff=time.monotonic()-.5
        with self.lock:recent=[s for t,s in self.history if t>=cutoff]
        if len(recent)<20:raise RuntimeError('Insufficient recent state samples')
        poses=np.array([[s.motor_state[i].q for i in range(29)] for s in recent])
        if not np.isfinite(poses).all() or np.max(np.ptp(poses,axis=0))>.025:
            raise RuntimeError('Robot has not been stationary for the last 0.5s')

    def publications(self):
        samples=self.discovery.read(2000,condition=self.alive)
        if len(samples)>=2000:raise RuntimeError('DDS publication inventory truncated; refuse ambiguous ownership')
        return {topic:{str(s.key) for s in samples if s.sample_info.valid_data and s.topic_name==topic}
                for topic in ('rt/lowstate','rt/arm_sdk')}

    def _discovery_inventory(self):
        publications=self.discovery.read(5000,condition=self.alive)
        participants=self.participant_discovery.read(5000,condition=self.participant_alive)
        subscriptions=self.subscription_discovery.read(5000,condition=self.subscription_alive)
        if any(len(x)>=5000 for x in (publications,participants,subscriptions)):
            raise RuntimeError('DDS discovery inventory truncated; refuse ambiguous ownership')
        participants={str(x.key):x for x in participants if x.sample_info.valid_data}
        pubs=[x for x in publications if x.sample_info.valid_data]
        subs=[x for x in subscriptions if x.sample_info.valid_data]
        return pubs,subs,participants

    def _endpoint_record(self,endpoint,pubs,subs,participants):
        participant=participants.get(str(endpoint.participant_key));properties={}
        if participant is not None:properties=_qos_properties(participant.qos)
        ips=_advertised_ips(properties);participant_key=str(endpoint.participant_key)
        related_pubs={x.topic_name for x in pubs if str(x.participant_key)==participant_key}
        related_subs={x.topic_name for x in subs if str(x.participant_key)==participant_key}
        expected=(properties.get('__Hostname')==EXPECTED_ARM_HOST and
                  properties.get('__ProcessName')==EXPECTED_ARM_PROCESS and
                  properties.get('__Pid')==self.expected_arm_pid and EXPECTED_ARM_IP in ips and
                  EXPECTED_ARM_PUBLICATIONS<=related_pubs and
                  EXPECTED_ARM_SUBSCRIPTIONS<=related_subs)
        own_pid=properties.get('__Pid')==str(os.getpid())
        own=(own_pid and (properties.get('__NetworkAddresses')=='localprocess' or
                          properties.get('__Hostname')==socket.gethostname()))
        handle=str(int(endpoint.sample_info.instance_handle))
        with self.lock:
            activity=dict(self.activity.get(endpoint.topic_name,{}).get('by_handle',{}))
        if expected:classification='EXPECTED ROBOT INTERNAL PARTICIPANT'
        elif own:classification='THIS DIAGNOSTIC/CONTROLLER PROCESS'
        else:classification='UNKNOWN EXTERNAL PARTICIPANT'
        return {'topic':endpoint.topic_name,'endpoint_guid':str(endpoint.key),
                'participant_guid':participant_key,'publication_handle':handle,
                'hostname':properties.get('__Hostname'),'process_name':properties.get('__ProcessName'),
                'pid':properties.get('__Pid'),'source_ips':ips,'classification':classification,
                'sample_count':int(activity.get(handle,0)),
                'related_publications':sorted(related_pubs),
                'related_subscriptions':sorted(related_subs)}

    def activity_snapshot(self):
        with self.lock:
            return {topic:{'count':int(value['count']),
                           'by_handle':{str(k):int(v) for k,v in value['by_handle'].items()},
                           'first_monotonic':dict(value['first_monotonic']),
                           'last_monotonic':dict(value['last_monotonic']),
                           'first_source_timestamp_ns':dict(value['first_source_timestamp_ns']),
                           'last_source_timestamp_ns':dict(value['last_source_timestamp_ns'])}
                    for topic,value in self.activity.items()}

    def ownership_snapshot(self,own_writer=False,allow_arm_action_unknown=False,
                           allow_robot_internal_idle_traffic=False):
        pubs,subs,participants=self._discovery_inventory()
        topics=(ARM_SDK_TOPIC,ARM_SDK_INPUT_TOPIC,ARM_ACTION_STATE_TOPIC,'rt/lowstate')
        records={topic:[self._endpoint_record(x,pubs,subs,participants)
                        for x in pubs if x.topic_name==topic] for topic in topics}
        arm=records[ARM_SDK_TOPIC]
        expected=[x for x in arm if x['classification']=='EXPECTED ROBOT INTERNAL PARTICIPANT']
        own=[x for x in arm if x['classification']=='THIS DIAGNOSTIC/CONTROLLER PROCESS']
        unknown=[x for x in arm if x['classification']=='UNKNOWN EXTERNAL PARTICIPANT']
        armsdk=records[ARM_SDK_INPUT_TOPIC]
        expected_active=any(x['sample_count']>0 for x in expected)
        with self.lock:
            action_messages=[x.copy() for x in self.action_messages]
            arm_command_messages=[x.copy() for x in self.arm_command_messages]
        action_message=action_messages[-1] if action_messages else None
        if action_message is None:
            action={'status':'UNKNOWN','reason':'no rt/arm/action/state sample observed'}
        else:
            action=decode_arm_action_state(action_message['data']);action.update(action_message)
            action['age_s']=time.monotonic()-action_message['received_monotonic']
            if action['age_s']>self.arm_action_stale_s:
                action['status']='UNKNOWN';action['reason']='latest action state is stale'
            elif any(decode_arm_action_state(x['data'])['status']=='ACTIVE'
                     for x in action_messages):
                action['status']='ACTIVE'
                action['reason']='an active Arm Action sample was observed during this session'
        expected_handles={x['publication_handle'] for x in expected}
        now=time.monotonic()
        recent_expected_commands=[x for x in arm_command_messages
                                  if x.get('publication_handle') in expected_handles and
                                  now-x['received_monotonic']<=.25]
        positive_expected_commands=[x for x in recent_expected_commands
                                    if x.get('weight',0.)>1e-6]
        expected_active_control=len(positive_expected_commands)>=2
        target_change=0.
        decoded=[x for x in recent_expected_commands if 'q' in x]
        if len(decoded)>=2:
            target_change=float(np.max(np.ptp(np.asarray([x['q'] for x in decoded]),axis=0)))
        reasons=[]
        if len(records['rt/lowstate'])!=1:reasons.append('LowState source is not exactly one')
        if len(expected)!=1:reasons.append('expected robot-internal arm service is not exactly one')
        if expected_active and not allow_robot_internal_idle_traffic:
            reasons.append('robot-internal rt/arm_sdk command traffic is active')
        if allow_robot_internal_idle_traffic and expected_active_control:
            reasons.append('robot-internal rt/arm_sdk positive-weight control is active')
        if unknown:reasons.append('unknown external rt/arm_sdk writer exists')
        if armsdk:reasons.append('rt/armsdk writer exists')
        reasons.extend(arm_action_ownership_reasons(
            action['status'],allow_arm_action_unknown))
        if own_writer:
            if len(own)!=1:reasons.append('this controller writer is not exactly one')
        elif own:
            reasons.append('local rt/arm_sdk writer exists before engagement')
        return {'passed':not reasons,'reasons':reasons,'expected_robot_internal':expected,
                'robot_internal_traffic':'ACTIVE' if expected_active else ('IDLE' if expected else 'UNKNOWN'),
                'robot_internal_active_command':expected_active_control,
                'robot_internal_recent_command_count':len(recent_expected_commands),
                'robot_internal_recent_positive_weight_count':len(positive_expected_commands),
                'robot_internal_recent_max_weight':max(
                    (x.get('weight',0.) for x in recent_expected_commands),default=0.),
                'robot_internal_target_change_rad':target_change,
                'robot_internal_idle_traffic_allowed':bool(allow_robot_internal_idle_traffic),
                'external_arm_sdk_writers':unknown,'own_arm_sdk_writers':own,
                'armsdk_writers':armsdk,'arm_action':action,
                'arm_action_telemetry_unknown':action['status']=='UNKNOWN',
                'arm_action_unknown_allowed':bool(allow_arm_action_unknown),
                'publications':records}

    def check_exclusive(self,own_writer=False,allow_arm_action_unknown=False,
                        allow_robot_internal_idle_traffic=False):
        ownership=self.ownership_snapshot(
            own_writer=own_writer,allow_arm_action_unknown=allow_arm_action_unknown,
            allow_robot_internal_idle_traffic=allow_robot_internal_idle_traffic)
        if not ownership['passed']:
            raise RuntimeError('Arm control ownership blocked: '+'; '.join(ownership['reasons']))
        return ownership

    def close(self):
        if self.lowstate_backend=='unitree':self.reader.Close()
        else:
            with self.lock:self.lowstate_closing=True
            del self.reader,self.lowstate_listener,self.lowstate_topic
        self.activity_readers.clear();self.activity_listeners.clear();self.activity_topics.clear()
        del self.subscription_alive,self.subscription_discovery
        del self.participant_alive,self.participant_discovery
        del self.alive,self.discovery,self.discovery_participant

def state_summary(state):
    return {'mode_machine':int(state.mode_machine),'mode_pr':int(state.mode_pr),'tick':int(state.tick),
            'imu_rpy':list(state.imu_state.rpy),'gyro':list(state.imu_state.gyroscope),
            'all_q':[float(m.q) for m in state.motor_state[:29]],
            'arm_q':[float(state.motor_state[i].q) for i in ARM_INDICES],
            'arm_dq':[float(state.motor_state[i].dq) for i in ARM_INDICES],
            'arm_motorstate':[int(state.motor_state[i].motorstate) for i in ARM_INDICES],
            'arm_temperature':[max(state.motor_state[i].temperature) for i in ARM_INDICES]}

class RecoverableMotionEnvelopeExceeded(RuntimeError):
    """First-test tilt/angular envelope violation; safety class is caller policy."""

    def __init__(self, *, tilt_peak, tilt_limit, gyro_peak, gyro_limit):
        self.tilt_peak = float(tilt_peak)
        self.tilt_limit = float(tilt_limit)
        self.gyro_peak = float(gyro_peak)
        self.gyro_limit = float(gyro_limit)
        super().__init__(
            'Robot tilt/angular motion exceeds first-test limits: '
            f'tilt_peak={self.tilt_peak:.6f}rad limit={self.tilt_limit:.2f}rad '
            f'gyro_peak={self.gyro_peak:.6f}rad/s limit={self.gyro_limit:.2f}rad/s')


class RecoverablePreCommandStationarityError(RuntimeError):
    """Arm velocity start gate failed before this caller sent any command."""


def validate_state(state,initial=False,hackathon_suspended_mode=False):
    s=state_summary(state)
    if s['mode_machine'] not in (2,3,5,6):
        raise RuntimeError(f"Unverified hardware variant mode_machine={s['mode_machine']}; requires 29DOF/7DOF arms")
    if s['mode_pr']!=0:raise RuntimeError('Need PR joint representation, mode_pr=0')
    validate_arm_pose(s['arm_q'])
    values=s['all_q']+s['arm_dq']+s['imu_rpy']+s['gyro']
    if not np.isfinite(values).all():raise RuntimeError('Non-finite robot state')
    tilt_limit=.30 if hackathon_suspended_mode else .20
    gyro_limit=.80 if hackathon_suspended_mode else .50
    tilt_peak=max(abs(x) for x in s['imu_rpy'][:2])
    gyro_peak=max(abs(x) for x in s['gyro'])
    if any(s['arm_motorstate']) or max(s['arm_temperature'])>=65:
        raise RuntimeError('Arm motor error or high temperature')
    if tilt_peak>tilt_limit or gyro_peak>gyro_limit:
        raise RecoverableMotionEnvelopeExceeded(
            tilt_peak=tilt_peak, tilt_limit=tilt_limit,
            gyro_peak=gyro_peak, gyro_limit=gyro_limit)
    if initial and max(abs(x) for x in s['arm_dq'])>.10:
        raise RecoverablePreCommandStationarityError('Arms are not stationary')
    return np.array(s['arm_q'])

def make_command(factory,crc,q,weight):
    if not np.isfinite(weight) or not 0<=weight<=1:raise ValueError('Invalid arm weight')
    validate_arm_pose(q)
    command=factory()
    # Same kp/kd, dq and tau as the official arm7 high-level example.
    # Leave every leg and waist field at its default (zero gains); no rt/lowcmd.
    for i in ACTIVE:
        index=ARM_INDICES[i]
        motor=command.motor_cmd[index]
        motor.q=float(q[i]);motor.dq=0.;motor.tau=0.;motor.kp=60.;motor.kd=1.5
    command.motor_cmd[29].q=float(weight)
    command.crc=crc.Crc(command)
    return command

class ArmWriter:
    def __init__(self,state):
        _,_,publisher,_,cmd,factory,crc=state.sdk
        self.factory=factory;self.crc=crc()
        self.publisher=publisher('rt/arm_sdk',cmd);self.publisher.Init()

    def write(self,q,weight):
        if self.publisher.Write(make_command(self.factory,self.crc,q,weight)) is False:
            raise RuntimeError('DDS write failed; robot receipt is not guaranteed')

    def release_now(self):
        # Fault path: zero all gains, zero blend weight. Do not move toward neutral.
        command=self.factory();command.motor_cmd[29].q=0.
        command.crc=self.crc.Crc(command)
        self.publisher.Write(command)

    def close(self):self.publisher.Close()
