"""Read a selected section from a cached public research source."""
import argparse
from pathlib import Path
import sys

sys.stdout.reconfigure(encoding="utf-8")
parser = argparse.ArgumentParser()
parser.add_argument("path", type=Path)
parser.add_argument("--after", default="")
parser.add_argument("--before")
parser.add_argument("--limit", type=int, default=16000)
args = parser.parse_args()
source = args.path.read_text(encoding="utf-8")
start = source.find(args.after)
if start < 0:
    raise SystemExit("Requested section not found")
end = source.find(args.before, start + len(args.after)) if args.before else -1
print(source[start:end if end >= 0 else None][:args.limit])
