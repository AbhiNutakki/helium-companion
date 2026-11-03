"""Exercise the real HTTP handler without starting Gazebo."""
import json, sys, threading, unittest, urllib.error, urllib.request
from pathlib import Path
from http.server import ThreadingHTTPServer
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import Handler


class FakeBridge:
    def __init__(self):
        self.commands = []
    def snapshot(self):
        return {'connected': True}
    def send(self, command):
        self.commands.append(command)
        return len(self.commands)


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        cls.server.bridge = FakeBridge()
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'
    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
    def request(self, body, origin=None):
        headers = {'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = origin
        req = urllib.request.Request(self.url+'/api/command', body, headers)
        try:
            response = urllib.request.urlopen(req, timeout=2)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            return response.code, json.load(response)
    def test_accepted_command_reaches_bridge(self):
        status, result = self.request(b'{"action":"mode","mode":"swim"}', self.url)
        self.assertEqual(status, 200)
        self.assertGreater(result['id'], 0)
        self.assertEqual(self.server.bridge.commands[-1], {'action': 'mode', 'mode': 'swim'})
    def test_bad_requests_never_reach_bridge(self):
        count = len(self.server.bridge.commands)
        for body in (b'{', b'{"action":[]}', b'{"action":"ballast"}',
                     b'{"action":"wings","left_hz":99}', b'x'*8193):
            with self.subTest(body=body[:60]):
                status, _ = self.request(body)
                self.assertEqual(status, 400)
        self.assertEqual(len(self.server.bridge.commands), count)
    def test_foreign_origin_rejected(self):
        count = len(self.server.bridge.commands)
        status, _ = self.request(b'{"action":"reset"}', 'https://example.com')
        self.assertEqual(status, 403)
        self.assertEqual(len(self.server.bridge.commands), count)
    def test_state_endpoint(self):
        with urllib.request.urlopen(self.url+'/api/state', timeout=2) as response:
            self.assertTrue(json.load(response)['connected'])

if __name__ == '__main__':
    unittest.main()
