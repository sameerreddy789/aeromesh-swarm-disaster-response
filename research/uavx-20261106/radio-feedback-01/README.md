# Transported image feedback and one-parity-chunk study

**Observed result:** selective retransmission can save bytes yet miss the ten-second deadline because its NACK must travel back. At 14 hops with a 0.5 s forwarding step, one missing chunk completes at 21 s under the tested NACK policy, compared with 9 s for periodic whole-image retransmission and 7.5 s for one upfront XOR parity chunk. XOR parity fails when two data chunks are missing. None of these policies solves arbitrary correlated loss.

This is an isolated research fixture, not a production protocol, flight run, RF calibration, reliability estimate or organizer benchmark. Production source and the earlier radio study are unchanged.

## Reproduce and interpret

- Script: [radio_feedback.py](../radio_feedback.py); raw results: [cases.jsonl](cases.jsonl); [manifest](manifest.json).
- Command: `.venv\Scripts\python.exe -B research\uavx-20261106\radio_feedback.py --output research\uavx-20261106\radio-feedback-01 --max-wall-seconds 115`.
- Run: **2026-11-06 05:27:32–05:27:36 UTC**, **80/80 cases**, **4.063 s** wall time.
- Grid: 5/14 hops; dt=0.5/0.1 s; four policies; five deterministic loss profiles; 32 KiB synthetic encoded-image bytes split into 64 chunks.
- Thirty UAV identities remain fixed. Selected chain radios use static links no longer than 100 m, 131,072 B/s shared half-duplex TX/RX service and 1 MiB per-node queues. Remaining identities are inactive. No flight, telemetry background, route discovery or physical RF interference is modeled.
- Canonical JSON/base64 envelope bytes, data retransmissions, reverse control packets and all link attempts consume the existing router's service. Each packet advances at most one hop per call. The configured hop delay changes with dt.
- The image deadline remains **10 s**; observations continue to **30 s**. The receiver verifies complete data length and SHA-256, including after XOR recovery. Later completion remains a deadline failure.

The four policies are one data offer with completion ACK; whole-image offers every 2 s until a transported completion ACK arrives; one selective NACK/repair round; and one data offer plus one upfront XOR parity chunk with completion ACK. ACK/NACK uses `PacketType.CONTROL` and therefore the router's highest scheduling priority. A received duplicate batch's last data chunk causes another ACK. Already queued duplicates are not cancelled after ACK receipt.

The NACK is issued when the last original data chunk arrives and lists currently missing indices. This is deliberately optimistic: the receiver may request still-in-flight data, a lost last chunk can prevent the trigger, and no additional NACK timeout or feedback-loss repair is provided. The experiment tests a timing proposition, not a complete reliable protocol.

The XOR packet contains the bytewise XOR of all 64 equal-length chunks. It can reconstruct **exactly one** missing data chunk if parity and the other 63 arrive; it cannot distinguish or recover two missing data chunks. Parity serialization and service are charged. This is not a general Reed–Solomon, fountain-code or multi-erasure FEC implementation.

## Feedback latency versus useful completion

One first-offer middle data chunk is erased on all four intrinsic source-link attempts; later application offers encounter no loss. All values are seconds; `>30` denotes no valid image at cutoff.

| Hops / dt | Policy | Complete image at GCS | NACK at source | Completion ACK at source |
|---|---|---:|---:|---:|
| 14 / 0.50 | Single offer | >30 | — | — |
| 14 / 0.50 | Whole-image retry | 9.0 | — | 16.0 |
| 14 / 0.50 | One NACK repair | 21.0 | 14.0 | 28.0 |
| 14 / 0.50 | XOR parity | 7.5 | — | 14.5 |
| 14 / 0.10 | Single offer | >30 | — | — |
| 14 / 0.10 | Whole-image retry | 4.0 | — | 5.4 |
| 14 / 0.10 | One NACK repair | 5.7 | 4.3 | 7.1 |
| 14 / 0.10 | XOR parity | 2.9 | — | 4.3 |
| 5 / 0.50 | Whole-image retry | 4.5 | — | 7.0 |
| 5 / 0.50 | One NACK repair | 7.5 | 5.0 | 10.0 |
| 5 / 0.50 | XOR parity | 3.0 | — | 5.5 |

At 14 hops and dt=0.5, the receiver detects the missing original chunk at 7 s, the NACK reaches the source at 14 s, and the repair reaches the receiver at 21 s. Even this favorable feedback detector cannot make that round trip fit a 10 s image deadline. An earlier NACK before the first received image data would require additional prior knowledge or an explicit timer model, and would not erase the 14 s source-to-receiver-to-source path cost.

Useful receipt and source knowledge are separate metrics. The receiver can hold a valid image before the source knows to stop retransmitting. A claim that a policy meets a ten-second image deadline must identify whether the deadline is receiver receipt or confirmed source knowledge; the current mission requires receipt, so a late ACK does not retroactively invalidate a timely image.

## Cost at 14 hops and dt=0.5

Totals include data and control offers over the 30 s observation window. Offered bytes count one serialized envelope per application offer; TX service bytes additionally count hops and repeated link attempts. RX service is charged separately by the same node calendars.

| Profile / policy | Complete image | Offered bytes | Of which control bytes | TX service bytes |
|---|---:|---:|---:|---:|
| No loss / single | 7.0 s | 64,994 | 246 | 909,916 |
| No loss / whole-image retry | 7.0 s | 455,506 | 1,734 | 6,377,084 |
| One erasure / whole-image retry | 9.0 s | 520,393 | 1,735 | 7,275,382 |
| One erasure / one NACK repair | 21.0 s | 66,264 | 503 | 917,576 |
| One erasure / XOR parity | 7.5 s | 66,035 | 246 | 914,370 |
| Two erasures / one NACK repair | 21.0 s | 67,280 | 506 | 921,680 |
| Two erasures / XOR parity | >30 | 65,789 | 0 | 900,806 |

Without loss, whole-image retry sends seven full batches before its first completion ACK reaches the source at 14 s. This is more traffic than a shared-state oracle would allow. A selective policy saves capacity but is not automatically the fastest policy; a small upfront parity cost is useful only within its declared erasure envelope.

## A packetization cliff, not a protocol speedup

The [first study](../radio-sensitivity-01/README.md) used 66,540 serialized bytes for a 32 KiB frame and observed 8 s no-loss completion at 14 hops/dt=0.5. This follow-up uses a deliberately explicit but different metadata envelope totaling **64,748 data bytes**, so no-loss completion is **7 s**. One 0.5 s radio interval supplies **65,536 bytes** at the configured rate. These envelopes lie on opposite sides of that service threshold.

The 8-to-7 s difference must not be credited to ACKs, parity, better RF, or payload compression. Both raw payloads are 32 KiB; metadata/serialization and scheduling changed the result. This exposes why image admission must use actual serialized bytes, deadlines, hop stages and competing traffic rather than only a nominal image-size label. Calibrating or decoupling the numerical service clock remains higher priority than claiming an unmeasured physical-network gain.

## Correlated blockage and ACK-only loss

Two profiles force every completed link attempt to fail while `2<=t<5`: either on the middle bidirectional edge, or on every chain edge. Graph membership and cached link quality remain connected, so the router does not receive perfect outage truth. The loss is sampled at attempt completion; this does not integrate time-varying physical SINR over a packet.

- At **14 hops/dt=0.5**, the all-link burst destroys all original data. Whole-image retry eventually completes at **11.5 s**, missing the deadline; one offer, one NACK round and XOR parity remain incomplete at 30 s. The NACK policy's tail-trigger never arrives.
- At **14 hops/dt=0.1**, the same scheduled all-link burst permits full-retry completion at **7.2 s** and ACK at **8.6 s**. Other tested policies still fail. Timing relative to the burst matters; these are not equal-mean-loss IID comparisons.
- At **5 hops/dt=0.1**, the middle-edge burst allows complete image receipt at **2.1 s** but loses the completion ACK. Single, NACK and parity modes never obtain source confirmation by 30 s. Whole-image retries trigger later ACKs and obtain confirmation at **7.2 s**. Thus successful GCS receipt, missing ACK and failed aircraft are observably different situations.
- At **14 hops/dt=0.5**, the middle-edge burst is short enough relative to the attempt cadence that existing intrinsic link retries preserve the original image, completing at **8.5 s**. At dt=0.1, those same four attempts can be exhausted while the three-second burst persists. A retry-count bound is not a fixed outage-tolerance duration.

These deterministic profiles demonstrate counterexamples and protocol tradeoffs. They do not estimate real outage probabilities, show urban link realism, or establish that XOR is optimal. Larger coding blocks, interleaving, selective repeat, adaptive repair and deadline-aware admission remain research candidates whose bytes, timing, reverse traffic and uncertainty must be charged.

## Verification

Every case checks offered=delivered+dropped+pending and verifies that no node consumes more than its interval service capacity. Image completion checks exact bytes and SHA-256. The manifest records unchanged before/after hashes for the production router, core models, both research scripts and the first study's raw results. No production test or mission was rerun as part of this experiment.
