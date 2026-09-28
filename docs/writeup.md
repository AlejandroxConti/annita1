# Annita-1: Vision Front-End for Flight Computer

**A satellite-grade image triage pipeline on a COTS crossover MCU: filtering what's worth sending home before it ever reaches the downlink.**

*(See attached: `annita1_cubesat_mounting.svg` — conceptual placement inside a 3U CubeSat bus, and `annita1_pipeline_diagram.svg` — full architecture. Add your own photos here: the complete physical setup with both boards connected, the camera module, and a close-up of the phyBOARD-RT1170.)*

## The problem

Every pixel a satellite sends to ground costs power, bandwidth, and ground-station time — resources that are always scarce and often the actual bottleneck of a mission, not the camera itself. Most small satellites still capture first and decide later, on the ground, days after the opportunity to act on the data has passed.

Annita-1 flips that order. It's an onboard vision front-end that looks at every frame *before* it's transmitted, scores how valuable it actually is, and decides right there — discard it, compress it, or send it in full — using a Science Value Score (SVS) computed from image quality, a lightweight neural classifier, and transmission cost.

## Why phyBOARD-RT1170 (and why not rad-hard)

Rad-hardened flight computers exist for good reason, but they're expensive, slow to iterate on, and usually years behind commercial silicon in raw compute. For CubeSats and many small-sat missions flying in relatively benign orbits with adequate shielding and error-correcting memory, that trade-off is increasingly being questioned — commercial off-the-shelf (COTS) processors are already flying, and the compute they bring is hard to ignore when the mission is compute-bound rather than radiation-bound.

The i.MX RT1176 on the phyBOARD-RT1170 is a strong fit for that argument: a real dual-core crossover MCU (Cortex-M7 + Cortex-M4) with a dedicated hardware image co-processor (PXP), enough headroom to run a quantized neural network on-device, and it's available *today*, off the shelf, at a fraction of the cost and lead time of a space-qualified part. Annita-1 is our attempt to show that a serious onboard decision pipeline — not a toy demo — is realistic on hardware like this.

## How it works

*(See `annita1_pipeline_diagram.svg`)*

An important note on the camera: the ESP32-S3 + OV5640 module is a **stand-in sensor**, not the point of the project. Annita-1's actual contribution — the decision pipeline — is sensor-agnostic. That's also why the onboard classifier is trained on real Sentinel-2 satellite imagery (the EuroSAT dataset), not on photos taken by the camera itself. The camera exists to prove the pipeline works against a live, asynchronous, imperfect data source, the way a real payload would.

**On the RT1170 itself:**
- **M7 core** runs the full pipeline: TJpgDec JPEG decode, PXP-accelerated rotation, an image quality score (sharpness via Laplacian variance + contrast), classical features (RGB565 histograms, Sobel edge density, ROI blue-dominance), a quantized TinyML CNN (98.44% int8 test accuracy on EuroSAT water-vs-land, TFLite Micro + CMSIS-NN), and the final SVS decision engine.
- **M4 core** owns UART acquisition from the camera — receiving and validating the framed JPEG protocol (`0xAA 0x55 [len] [payload] [checksum]`) independently of the M7's compute load, and handing off complete, checksum-verified frames through shared OCRAM.

## Results

These are measured numbers from real hardware runs, not simulated targets:

| Metric | Result |
|---|---|
| JPEG compression on-device | 307,200 B raw → ~3,500–3,700 B (~83×), ~135 ms/frame |
| Decode + PXP + IQA + features + TinyML | ~10–18 ms decode, ~65 ms IQA, ~15 ms features, ~12 ms inference |
| 80-frame transmission-decision run | 75 discarded / 5 compressed / 0 full-transmit — **254,921 bytes saved vs. 8,408 transmitted (~96% of downlink avoided)** |
| 415-frame, ~19.6-minute stability run | No timing drift or gaps in any stage; **1,684,392 bytes saved**, 317 discarded, 97 compressed, over multiple real land/water scene changes presented to the camera during the run |
| M4↔M7 crossover (acquisition/compute split) | M4 receives real camera JPEGs over its own UART and hands off validated frames to the M7 via shared memory; ~75–80% frame success in sustained live testing, zero core lockups |

You can see a real, second-by-second replay of the 415-frame stability run — including live quality scores, classifier confidence, and the SVS decision — here: **[Annita-1 Telemetry Console](https://<github-user>.github.io/<repo>/console/)**. Every number in it is pulled from the actual captured log, not staged.

## Engineering trade-offs

Two decisions are worth calling out:

- **We chose reliability over frame-completeness on the M4's UART link.** An interrupt-driven receiver looked like the natural first choice, but testing revealed a scheduler interaction that could stall the core under sustained load. We replaced it with a polling receiver that trades a modest frame-loss margin (roughly 20–25% under sustained load) for zero lockups — the right call for a flight-adjacent design where uptime matters more than catching every single frame.
- **Full closed-loop integration** — the M4's acquisition path feeding the M7's decision pipeline in one continuous run — is built and running on both cores independently; wiring them into a single always-on pipeline is our next milestone.

## COTS-based onboard AI: pros and cons

**Pros**
- Order-of-magnitude more compute and memory than typical rad-hard flight processors, at a fraction of the cost and lead time.
- A real hardware image co-processor (PXP) and enough headroom for on-device ML inference — both out of reach on many legacy space-grade MCUs.
- Fast iteration: the entire pipeline above was built, measured, and revised on a single dev kit in real dev cycles, not multi-month radiation-qualification cycles.

**Cons**
- No inherent radiation tolerance — viable mainly for shorter missions, benign orbits, or designs that accept redundancy/voting at the system level instead of at the silicon level.
- Long-term flight heritage and qualification data are far thinner than for rad-hard parts, which matters for mission insurance and review boards.
- Dual-core coordination adds real firmware complexity that a single-core rad-hard design doesn't have to deal with.

## What's next

- Close the loop on M4↔M7 full-pipeline integration and reduce the remaining frame-loss margin on the M4's UART receiver.
- Add a 4th SVS factor using mission-state variables (battery margin, remaining ground-contact window) — a natural extension once the M4 is fully free to own real-time telemetry.
- Extend the TinyML target beyond water/land (cloud detection has real precedent in CubeSat literature and was deprioritized only for time, not feasibility).

---

Annita-1 was built for the Phytec/NXP phyBOARD-RT1170 Design Contest.
