"""Validate local research records and links; does not rerun simulations."""
from pathlib import Path
import ast
import hashlib
import json
import math
import re
import subprocess
import sys
from urllib.parse import unquote

sys.stdout.reconfigure(encoding="utf-8")
root = Path(__file__).resolve().parents[1]
repository = root.parents[1]
errors, warnings = [], []
ids = set()
for name in ("root-sources.json", "physics-control-sources.json",
             "swarm-network-sources.json", "visual-demo-sources.json"):
    data = json.loads((root / name).read_text(encoding="utf-8"))
    records = data if isinstance(data, list) else data["sources"]
    for record in records:
        if record["id"] in ids:
            errors.append("Duplicate source " + record["id"])
        ids.add(record["id"])

def finite(value, where):
    if isinstance(value, float) and not math.isfinite(value):
        errors.append("Nonfinite value " + where)
    elif isinstance(value, dict):
        for key, item in value.items():
            finite(item, where + "/" + key)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            finite(item, where + "/" + str(index))

checked_json = checked_lines = checked_links = checked_python = 0
portable_links = set()
for path in root.rglob("*"):
    if not path.is_file() or any(part in ("raw", "__pycache__", "radio-pilot-01", "radio-feedback-pilot-01") for part in path.parts):
        continue
    if path.name == "research-audit.json":
        continue
    try:
        if path.suffix == ".py":
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            checked_python += 1
        elif path.suffix == ".json":
            finite(json.loads(path.read_text(encoding="utf-8")), str(path.relative_to(root)))
            checked_json += 1
        elif path.suffix == ".jsonl":
            for index, line in enumerate(path.read_text(encoding="utf-8").splitlines()):
                finite(json.loads(line), f"{path.name}:{index+1}")
                checked_lines += 1
        elif path.suffix == ".md":
            text = path.read_text(encoding="utf-8")
            for sid in set(re.findall(r"\b(?:SW|P|R|V)\d{2}\b", text)):
                if sid not in ids:
                    errors.append(f"Unknown citation {sid} in {path.name}")
            for target in re.findall(r"\]\(([^)]+)\)", text):
                target = target.strip("<>")
                if re.match(r"(?:https?://|mailto:|#)", target):
                    continue
                target = unquote(target.split("#")[0])
                if not target:
                    continue
                candidate = Path(target) if re.match(r"^[A-Za-z]:[/\\]", target) else path.parent / target
                checked_links += 1
                if not candidate.exists():
                    errors.append(f"Missing local link {target} in {path.relative_to(root)}")
                else:
                    try:
                        portable_links.add(candidate.resolve().relative_to(repository).as_posix())
                    except ValueError:
                        warnings.append(f"Machine-local link {target} in {path.relative_to(root)}")
    except (UnicodeError, ValueError, SyntaxError) as exc:
        errors.append(f"Invalid text/JSON {path.name}: {exc}")

ignored = subprocess.run(["git", "check-ignore", "--no-index", "--stdin"],
                         input="\n".join(sorted(portable_links))+"\n", cwd=repository,
                         capture_output=True, text=True, encoding="utf-8")
if ignored.returncode not in (0, 1):
    errors.append("Unable to check linked-file publication: " + ignored.stderr.strip())
for name in ignored.stdout.splitlines():
    errors.append("Delivered document links an ignored file: " + name)

copies = json.loads((root / "baseline-summaries/provenance.json").read_text(encoding="utf-8"))
for record in copies["records"]:
    path = root / "baseline-summaries" / record["saved_copy"]
    blob = path.read_bytes()
    if len(blob) != record["bytes"] or hashlib.sha256(blob).hexdigest() != record["sha256"]:
        errors.append("Changed baseline summary copy " + record["saved_copy"])

physics_validation = json.loads((root / "physics-research-validation.json").read_text(encoding="utf-8"))
for source, expected in physics_validation["frozen_source_sha256"].items():
    if hashlib.sha256((repository / source).read_bytes()).hexdigest() != expected:
        errors.append("Changed physics study source " + source)

studies = {}
for name, count in (("radio-sensitivity-01", 240), ("radio-feedback-01", 80)):
    folder = root / name
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    actual_count = sum(bool(line.strip()) for line in (folder / "cases.jsonl").read_text(encoding="utf-8").splitlines())
    studies[name] = actual_count
    if actual_count != count or manifest["completed_cases"] != count or not manifest["source_stable"]:
        errors.append("Study count/source mismatch " + name)
    for source, expected in manifest["source_sha256_after"].items():
        path = repository / source
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            errors.append("Changed study input/source " + source)

result = {"scope": "Research syntax, local links, citation IDs, study counts and source hashes; not scientific peer review or mission qualification",
          "source_records": len(ids), "python_files": checked_python,
          "json_files": checked_json, "jsonl_rows": checked_lines,
          "local_links": checked_links, "studies": studies, "errors": errors, "warnings": warnings,
          "passed": not errors}
(root / "research-audit.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, indent=2))
raise SystemExit(0 if not errors else 1)
