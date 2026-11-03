#!/usr/bin/env python3
"""Run unit and Gazebo acceptance checks in an isolated, owned session."""
from pathlib import Path
import subprocess, sys
from session import ROOT, simulation

if __name__ == '__main__':
    reports = ROOT/'reports'
    reports.mkdir(exist_ok=True)
    unit = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s',
                           str(ROOT/'tests'), '-p', 'test_*.py', '-v'],
                          capture_output=True, text=True)
    output = unit.stdout + unit.stderr
    print(output, end='', flush=True)
    (reports/'unit-tests.txt').write_text(output)
    if unit.returncode:
        sys.exit(unit.returncode)
    with simulation() as url:
        tests = subprocess.run([sys.executable, str(ROOT/'tests/integration.py'), '--url', url])
    sys.exit(tests.returncode)
