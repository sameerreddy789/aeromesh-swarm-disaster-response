# Live AeroMesh simulation — 5 November 2026

`Start-AeroMesh.cmd` opens this repository's updated UAV-X simulation in a native
pygame window titled **AeroMesh UAV-X | Live Swarm Simulation**. It uses
`simulation/resilient_uavx.py` with the shared `simulation/uavx.py` engine; no DroneOS pilot, separate city environment or hardware
connection is involved. The map is a live 2D view of 3D kinematic coordinates.
Current scenario-camera results and exact environment checks are recorded in
[current validation](UAVX_VALIDATION.md); the older measurements below remain
labelled as baseline evidence.

## Launch

Double-click `Start-AeroMesh.cmd` from the repository root on Windows. It uses
Python 3.10, creates `.venv` if absent, and installs the exact `requirements-repro.txt` versions if the
GUI dependencies are missing. Internet access is needed for that first install.
The default scenario explicitly uses 60 UAVs, seed 42, a Panda3D scene camera, protected relay spacing, no injected fault and 12x requested
playback. Actual speed depends on the host; model speed remains limited to 5 m/s.

For a slower launch or a fault demonstration:

```powershell
.\Start-AeroMesh.cmd --playback-speed 4
.\Start-AeroMesh.cmd --scenario-file scenarios/uavx/radio-outage.json
.\Start-AeroMesh.cmd --scenario-file scenarios/uavx/permanent-failure.json
```

Each launcher invocation chooses a fresh evidence directory. The equivalent
direct command requires a new output path:

```powershell
.venv\Scripts\python.exe run_simulation.py --profile uavx --scenario-file scenarios/uavx/nominal.json --gui --playback-speed 4 --output evidence/my-live-run
```

## Controls and interpretation

| Control | Action |
|---|---|
| Space or Pause button | Pause/resume model stepping |
| + / - | Change requested wall-clock playback speed |
| L / T | Toggle actual radio links / position trails |
| Mouse wheel / right-drag / Home | Zoom / pan / fit map |
| V | Switch received camera / UAV details panels |
| Click UAV | Show its role, altitude, battery and GCS path |
| R after completion | Start another mission in a numbered output directory |
| Esc or window close | End the run and write its current evidence |

PoIs appear at their configured emergence time. Green PoIs have been surveyed
and have a GCS receipt. The network count covers airborne UAVs; the selected
UAV's GCS path may traverse several hops. A line is drawn only for an actual
topology edge. Paths and trails are top-down projections, so overlapping lines
alone do not show a 3D collision.

The window stays open at mission end. `--auto-close` closes after one run;
`--repeat` starts another after a four-second result display. These flags are
mutually exclusive. Closing early records incomplete checks rather than success.

## Preserved baseline validation and saved evidence

The measurements below describe the earlier 30-UAV footprint profile. The new
scenario/camera profile is documented in [resilience notes](UAVX_RESILIENCE.md);
its current matrix has separate source fingerprints and evidence.

A complete native-window seed-42 run finished at **1906.0 simulated seconds**,
with **10/10 PoIs surveyed and received, all 30 UAVs landed, and all 12 sample
checks passing**. Advancing timestamps and changing positions were observed
while it ran. Its verifier reported no evidence errors. Three further full
headless runs preserve the expected seed-7 success and strict fault failures.
The full targeted suite passed **172 tests**, including byte-identical core
evidence between the headless and GUI versions of a short mission.

`Start-AeroMesh.cmd --max-time 2 --auto-close --playback-speed 100` also opened,
ran and closed successfully; its two-second mission correctly remained incomplete.
Dependency checking reported no broken requirements.

Run artifacts remain `metrics.json`, `events.json`, `packets.json`,
`connectivity.json`, `trajectory.jsonl` and `manifest.json`. The separate
`viewer/` directory contains `live.json`, periodic `live.png`, checkpoint PNGs
and `final.png`. Viewer snapshots are visual diagnostics, outside the five core
artifacts hashed by the run manifest. Full local runs are ignored by Git.

Current compact evidence is in
[gui-benchmark-summary.json](verification/uavx/gui-benchmark-summary.json).
Full commands and measurements are in [validation](UAVX_VALIDATION.md).

## Current scope

The scenario profile adds rendered camera images and bounded per-node radio
service. Its camera panel displays complete GCS-received PNGs only. It does not
add flight physics or physical-camera qualification. Complete endpoint-radio
outages and permanent aircraft failures retain their strict failure meanings.

## Preserved baseline limits

This live view does not add flight physics, camera detection, distributed
middleware or bounded radio bandwidth. The 30 s sole-radio outage still fails
continuous connectivity. A permanently frozen UAV cannot land, so that scenario
also fails all-UAV landing and the mission deadline. All three runs are useful
demonstrations, but only the nominal cases pass the implemented sample checks.
See the [failure register](UAVX_FAILURES.md) for competition and submission gaps.
