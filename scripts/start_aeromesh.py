"""Launch this checkout's live UAV-X simulation into a fresh evidence folder."""
from datetime import datetime
from pathlib import Path
import subprocess
import sys
from uuid import uuid4

root=Path(__file__).resolve().parents[1]
output=root/'evidence'/f'live-{datetime.now():%Y%m%d-%H%M%S}-{uuid4().hex[:6]}'
command=[sys.executable,str(root/'run_simulation.py'),'--profile','uavx','--gui',
         '--playback-speed','12','--seed','42','--output',str(output),*sys.argv[1:]]
raise SystemExit(subprocess.call(command,cwd=root))
