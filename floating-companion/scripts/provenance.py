"""Identify the exact control/model source used for a saved simulation run."""
import hashlib, subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


def source_info():
    digest = hashlib.sha256()
    files = sorted({*ROOT.glob('*.py'), *ROOT.glob('*.txt'),
                    *ROOT.glob('src/*.cc'), *ROOT.glob('scripts/*.py'),
                    *ROOT.glob('tests/*.py'), *ROOT.glob('assets/*.obj')})
    for file in files:
        digest.update(str(file.relative_to(ROOT)).encode() + b'\0' + file.read_bytes() + b'\0')
    info = {'source_sha256': digest.hexdigest()}
    try:
        info['git_commit'] = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True, stderr=subprocess.DEVNULL).strip()
        # Reports change as a run progresses, so record source modifications separately.
        changed = subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain', '--',
                                          '*.py', '*.cc', '*.txt', 'assets/*.obj', ':!reports'], text=True)
        info['source_worktree_modified'] = bool(changed.strip())
    except (OSError, subprocess.CalledProcessError):
        info['git_commit'] = None
    return info
