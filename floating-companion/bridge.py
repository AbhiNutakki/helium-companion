"""Loopback dashboard bridge for the helium/flapping Gazebo model."""
import json,threading,time
from gz.transport import Node
from gz.msgs.stringmsg_pb2 import StringMsg
class Bridge:
    def __init__(self):
        self.node=Node();self.lock=threading.RLock();self.state={};self.received=0.;self.sequence=0
        self.publisher=self.node.advertise('/companion/command',StringMsg)
        self.node.subscribe(StringMsg,'/companion/state',self.on_state)
    def on_state(self,message):
        try:
            state=json.loads(message.data)
            if not isinstance(state,dict):return
            ack=state.get('ack',0)
            if isinstance(ack,bool) or not isinstance(ack,int) or not 0<=ack<2147483647:return
            with self.lock:
                self.state=state;self.received=time.monotonic();self.sequence=max(self.sequence,ack)
        except (ValueError,TypeError):pass
    def send(self,command):
        with self.lock:
            if not self.received or time.monotonic()-self.received>=2:
                raise ConnectionError('Gazebo telemetry is disconnected; wait for reconnection')
            if self.sequence>=2147483646:raise ConnectionError('Command counter exhausted; restart the simulation')
            self.sequence+=1;message=StringMsg();message.data=json.dumps(dict(command,id=self.sequence),allow_nan=False)
            if not self.publisher.publish(message):raise ConnectionError('Gazebo command could not be published')
            return self.sequence
    def snapshot(self):
        with self.lock:return dict(self.state,connected=bool(self.received) and time.monotonic()-self.received<2)
