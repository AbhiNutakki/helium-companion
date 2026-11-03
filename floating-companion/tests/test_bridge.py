"""Telemetry and delivery failures must not look like applied commands."""
import json, sys, time, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge import Bridge


class BridgeTests(unittest.TestCase):
    def setUp(self):
        with patch('bridge.Node'):
            self.bridge = Bridge()
        self.bridge.publisher.publish.return_value = True
    def receive(self, value):
        self.bridge.on_state(SimpleNamespace(data=json.dumps(value)))
    def test_initially_disconnected(self):
        self.assertFalse(self.bridge.snapshot()['connected'])
        with self.assertRaises(ConnectionError):
            self.bridge.send({'action':'reset'})
        self.bridge.publisher.publish.assert_not_called()
    def test_invalid_telemetry_does_not_replace_state_or_refresh_connection(self):
        self.receive({'ack':4, 'mode':'float'})
        self.bridge.received = time.monotonic()-3
        for value in ([], None, 7, {'ack':-1}, {'ack':True}, {'ack':1.5}, {'ack':2147483647}):
            self.receive(value)
        self.bridge.on_state(SimpleNamespace(data='{broken'))
        self.assertEqual(self.bridge.snapshot()['ack'], 4)
        self.assertFalse(self.bridge.snapshot()['connected'])
    def test_reconnect_continues_after_simulator_ack(self):
        self.receive({'ack':42})
        self.assertEqual(self.bridge.send({'action':'reset'}), 43)
        sent = self.bridge.publisher.publish.call_args.args[0]
        self.assertEqual(json.loads(sent.data)['id'], 43)
    def test_old_telemetry_does_not_reuse_command_ids(self):
        self.receive({'ack':20})
        self.assertEqual(self.bridge.send({'action':'reset'}), 21)
        self.receive({'ack':19})
        self.assertEqual(self.bridge.send({'action':'reset'}), 22)
    def test_stale_connection_rejects_commands(self):
        self.receive({'ack':0})
        self.bridge.received -= 3
        with self.assertRaises(ConnectionError):
            self.bridge.send({'action':'reset'})
        self.bridge.publisher.publish.assert_not_called()
    def test_publish_failure_does_not_return_success(self):
        self.receive({'ack':0})
        self.bridge.publisher.publish.return_value = False
        with self.assertRaises(ConnectionError):
            self.bridge.send({'action':'reset'})
    def test_counter_cannot_overflow_simulator_integer(self):
        self.receive({'ack':2147483646})
        with self.assertRaises(ConnectionError):
            self.bridge.send({'action':'reset'})
        self.bridge.publisher.publish.assert_not_called()

if __name__ == '__main__':
    unittest.main()
