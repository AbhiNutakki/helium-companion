"""Own an isolated headless Gazebo session for tests and design experiments."""
from contextlib import contextmanager
from pathlib import Path
import json, math, os, socket, subprocess, sys, time, urllib.request, uuid

ROOT = Path(__file__).resolve().parents[1]

@contextmanager
def simulation(factor=6):
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError('factor must be positive and finite')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    partition = 'companion_experiment_' + uuid.uuid4().hex
    reports = ROOT / 'reports'
    reports.mkdir(exist_ok=True)
    logpath = reports / f'acceptance-{port}.log'
    url = f'http://127.0.0.1:{port}'
    env = os.environ.copy()
    env['PATH'] = '/opt/homebrew/bin' + os.pathsep + env.get('PATH', '')
    with logpath.open('w') as log:
        proc = subprocess.Popen([sys.executable, str(ROOT/'app.py'), '--port', str(port),
                                 '--factor', str(factor), '--partition', partition],
                                env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 40
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f'Simulator exited; see {logpath}')
                try:
                    with urllib.request.urlopen(url+'/api/state', timeout=1) as response:
                        state = json.load(response)
                except OSError:
                    state = {}
                if state.get('connected') and state.get('model_revision') == 'helium_flapping_v2' and state.get('session_partition') == partition:
                    # Do not accept an unrelated server if another process won the port race.
                    time.sleep(.1)
                    if proc.poll() is None:
                        yield url
                        return
                time.sleep(.2)
            raise RuntimeError(f'Simulator did not become ready; see {logpath}')
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
