"""Launch this checkout's live UAV-X simulation into a fresh evidence folder."""
from datetime import datetime
from pathlib import Path
import subprocess
import sys
from uuid import uuid4

def main():
    root=Path(__file__).resolve().parents[1]
    output=root/'evidence'/f'live-{datetime.now():%Y%m%d-%H%M%S}-{uuid4().hex[:6]}'
    extra=sys.argv[1:]
    # Explicit legacy disturbance requests retain their original semantics.
    # Otherwise the one-click demonstration runs the camera/resilience profile.
    scenario=[] if any(arg.split('=')[0] in ('--scenario-file','--disturbance') for arg in extra) else [
        '--scenario-file',str(root/'scenarios/uavx/nominal.json')]
    command=[sys.executable,str(root/'run_simulation.py'),'--profile','uavx','--gui',
             '--playback-speed','12','--output',str(output),*scenario,*extra]
    return subprocess.call(command,cwd=root)


if __name__=='__main__':
    raise SystemExit(main())
