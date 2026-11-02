#!/usr/bin/env python3
"""Articulated helium/flapping acceptance suite in an isolated Gazebo partition."""
from pathlib import Path
import os,subprocess,sys,time,urllib.request,json
ROOT=Path(__file__).resolve().parents[1];PY=ROOT/'.venv/bin/python'
if __name__=='__main__':
    unit=subprocess.run([str(PY),'-m','unittest','discover','-s',str(ROOT/'tests'),'-p','test_*.py','-v']).returncode
    with open(ROOT/'reports/acceptance-8875.log','w') as log:
        proc=subprocess.Popen([str(PY),str(ROOT/'app.py'),'--port','8875','--factor','6','--partition',f'flapping_acceptance_{os.getpid()}'],stdout=log,stderr=subprocess.STDOUT)
        try:
            for _ in range(150):
                if proc.poll() is not None:raise RuntimeError('Gazebo exited: see acceptance-8875.log')
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8875/api/state',timeout=1) as r:ready=json.load(r).get('connected')
                    if ready:break
                except OSError:pass
                time.sleep(.2)
            else:raise RuntimeError('Gazebo did not become ready')
            tests=subprocess.run([str(PY),str(ROOT/'tests/integration.py'),'--url','http://127.0.0.1:8875']).returncode
        finally:
            proc.terminate()
            try:proc.wait(timeout=12)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
    sys.exit(max(unit,tests))
