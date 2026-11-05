# IIT Bombay UAV-X: remaining implementation and submission work

The current scope is the fixed **30-UAV core**. City/Unreal expansion is paused
at the user's request because of storage. Existing files remain preserved;
reuse the existing environment and avoid more city downloads.

The current implementation provides configurable faults, protected relay
geometry, budgeted formation admission, measured handoffs/landing, bounded
radio service and rendered image delivery. See [resilience](UAVX_RESILIENCE.md)
for the implementation and [validation](UAVX_VALIDATION.md) for actual outcomes.
An implemented feature is not automatically a qualified mission capability.

## Preserve the completed core evidence

The final source has completed the 19-case matrix and 486-test core suite.
All 12 nominal seeds and the link/resource/packet-loss cases passed. The two
strict fault failures remain recorded with valid, source-stable evidence.

The eight-page report, 74.4 s demo, all-case validation data and compressed
evidence archive are prepared and visually/integrity checked at
`submission-artifacts/fleet30-core-20261106`. See
[verified package contents](SUBMISSION_PACKAGE.md). The final committed tree
is packaged as `source.zip`; the requested draft PR provides code review.
Preserve the source snapshot and all earlier attempts. Further runtime changes
need fresh source-matching results.

Strict scoring remains unchanged: a sole-radio outage disconnects its endpoint,
and permanent aircraft loss cannot satisfy all-UAV landing. Survivor
performance stays separate from those original checks.

The already-running development model remains useful for the challenge's
mission, communication and reconfiguration algorithms. It does not establish
physics, RF or airframe calibration. Hidden organizer scenarios must be run
unchanged when supplied; the current declared seeds are development tests.

## Confirm participation and submission requirements

The supplied schedule closed Stage 1 on 27 September. Stage 2 runs from
3 October to 2 December 2026 for qualified teams. Team registration and
qualification have not been established from the available material.

The supplied challenge PDF p. 8 specifies `pushpak_gc2026@aero.iitb.ac.in` and
says no submission links are used. The Forms URL in the user's original message
conflicts with that instruction. The team needs the currently applicable route,
official advanced scenarios and required log schema before final submission.
No organizer email or form submission has been sent.

Additional owner/organizer decisions remain:

- Team name, member/institution details, registration ID, qualification and
  contributions for the report cover.
- Acceptance of 30 UAVs, staging/charging resources, the custom framework and
  the intended scoring of permanent failures and communication outages.
- The root-project license, which is absent despite historical MIT claims.
  Third-party dependencies retain their own terms.

These missing inputs do not require stopping authorized core engineering;
they do prevent claiming organizer qualification or a completed submission.

## Resume physical simulation later

The supplied challenge PDF p. 5 and report template section 9 allow an
open-source framework such as PX4/ArduPilot SITL, Gazebo, AirSim, Webots, ns-3,
or equivalent. Acceptance of the current custom kinematic model as equivalent
is unconfirmed. A particular city engine is not a supplied requirement.

When storage and framework requirements are resolved, qualify one backend
with measured multi-UAV motion/sensing and then scale to 30 identities. Planner
commands must drive the backend and actual telemetry must drive the planner.
Command acknowledgements and pose animation cannot establish physics flight.
The Garuda single-client scene reload behavior is not a proven swarm adapter.

The report template allows centralized, decentralized or hybrid coordination
with justification. The core's centralized architecture should therefore be
explained accurately. The mission requires imagery/situational data at PoIs;
general learned survivor recognition is not established by the current
synthetic fixture and should not be claimed.
