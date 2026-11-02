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
            with self.lock:self.state=json.loads(message.data);self.received=time.monotonic()
        except (ValueError,TypeError):pass
    def send(self,command):
        with self.lock:
            self.sequence+=1;message=StringMsg();message.data=json.dumps(dict(command,id=self.sequence),allow_nan=False);self.publisher.publish(message);return self.sequence
    def snapshot(self):
        with self.lock:return dict(self.state,connected=time.monotonic()-self.received<2)
