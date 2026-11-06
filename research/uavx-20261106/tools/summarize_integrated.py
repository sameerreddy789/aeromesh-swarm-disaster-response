"""Summarize a completed nominal research run; never modifies its evidence."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

sys.stdout.reconfigure(encoding="utf-8")
parser = argparse.ArgumentParser()
parser.add_argument("folder", type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
folder = args.folder.resolve()
required = ("metrics.json", "manifest.json", "events.json", "physics-safety.json", "trajectory.jsonl")
if not all((folder / name).exists() for name in required):
    raise SystemExit("Run is not finalized; no summary or plot generated")
read = lambda name: json.loads((folder / name).read_text(encoding="utf-8"))
metrics, manifest, events, safety = (read(name) for name in required[:4])
verification_path = root / "integrated-nominal-verification.json"
if not verification_path.exists():
    raise SystemExit("Run the independent physics verifier first")
verification = json.loads(verification_path.read_text(encoding="utf-8"))
trajectory = [json.loads(line) for line in (folder / "trajectory.jsonl").read_text(encoding="utf-8").splitlines()]
takeoffs = sorted({event["uav_id"] for event in events if event["type"] == "takeoff_started"})
fields = ("seed", "fleet_size", "completion_time_s", "elapsed_time_s", "pois_completed", "pois_total",
          "minimum_separation_m", "maximum_speed_mps", "maximum_altitude_m", "maximum_sortie_s",
          "communication_downtime_s", "battery_exhaustions", "collision_count", "checks",
          "sample_constraints_passed", "source_stable", "reporting_deadline_misses")
summary = {key: metrics[key] for key in fields}
summary.update(scope="Completed team-authored nominal Bullet scenario; not hidden/fault matrix or airframe qualification",
               summarized_utc=datetime.now(timezone.utc).isoformat(),
               evidence_folder=str(folder), source_sha256=manifest["source_sha256"],
               manifest_sha256=hashlib.sha256((folder / "manifest.json").read_bytes()).hexdigest(),
               distinct_launched_uavs=takeoffs, never_launched_count=metrics["fleet_size"]-len(takeoffs),
               peak_sampled_active=max(sample["active"] for sample in safety),
               verification=verification, scenario=metrics["scenario"])
(root / "integrated-nominal-summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
fig, axes = plt.subplots(4, 1, figsize=(11, 11), constrained_layout=True)
minutes = [sample["time_s"]/60 for sample in trajectory]
axes[0].plot(minutes, [sum(u["armed"] for u in s["uavs"]) for s in trajectory], color="#126d82", label="Armed (2 s trace)")
axes[0].plot(minutes, [sum(u["role"].lower()=="charging" for u in s["uavs"]) for s in trajectory], color="#a85e13", label="Charging (2 s trace)")
axes[0].set(ylabel="Aircraft count", xlabel="Simulated minutes", ylim=(-1,31))
axes[0].legend(loc="upper right", frameon=False)
axes[0].set_title("Fleet activity — 30 total including spares", loc="left")
axes[1].plot([s["time_s"]/60 for s in safety], [s["minimum_separation_m"] for s in safety], color="#126d82", linewidth=.8)
axes[1].axhline(20, color="#b23b3b", linestyle="--", label="Required separation: 20 m")
axes[1].set(xlabel="Simulated minutes", ylabel="Minimum separation (m)")
axes[1].set_title("Recorded separation at 20 Hz — not a continuous-time proof", loc="left")
axes[1].legend(frameon=False)
tasks = sorted(metrics["tasks"], key=lambda task: task["id"])
axes[2].plot([s["time_s"]/60 for s in safety], [s["active"]-s["connected"] for s in safety], color="#b23b3b", linewidth=.8)
axes[2].set(xlabel="Simulated minutes", ylabel="Active without GCS path")
axes[2].set_title(f"Failing check: continuous connectivity — {metrics['communication_downtime_s']:.1f} s recorded downtime", loc="left")
for index, task in enumerate(tasks):
    delay = None if task["report_s"] is None or task["detection_s"] is None else task["report_s"]-task["detection_s"]
    if delay is None:
        axes[3].plot(index, 10, "x", color="#b23b3b", markersize=9)
        axes[3].annotate("No receipt", (index,10), rotation=90, xytext=(0,5), textcoords="offset points", ha="center", fontsize=8)
    else:
        axes[3].bar(index, delay, color="#126d82" if delay<=10 else "#b23b3b")
axes[3].axhline(10, color="#b23b3b", linestyle="--", label="10 s deadline")
axes[3].set(xticks=range(len(tasks)), xticklabels=[str(task["id"]) for task in tasks], xlabel="PoI ID", ylabel="Complete receipt delay (s)")
axes[3].set_ylim(0,max(13,axes[3].get_ylim()[1]))
axes[3].legend(frameon=False)
axes[3].set_title("First detection to validated complete image receipt", loc="left")
for axis in axes:
    axis.grid(axis="y", alpha=.15)
outcome = "PASS" if verification["mission_passed"] else "DOES NOT PASS"
fig.suptitle(f"AeroMesh nominal physics, seed {metrics['seed']} — {outcome}\nMeasured run summary; uncalibrated model", fontsize=14, ha="left", x=.02)
fig.savefig(root/"integrated-nominal-timeline.png", dpi=160)
fig.savefig(root/"integrated-nominal-timeline.svg")
plt.close(fig)
print(json.dumps({key:summary[key] for key in ("elapsed_time_s", "pois_completed", "sample_constraints_passed", "source_stable", "peak_sampled_active", "never_launched_count")}, indent=2))
