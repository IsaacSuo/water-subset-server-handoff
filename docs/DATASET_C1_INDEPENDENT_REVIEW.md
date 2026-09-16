# C1 independent-episode reproduction

Completed locally on 2026-09-16. This closes the existing translational probes' packaging
and isolation check, **not C1 as a whole and not any C2 prototype**. No video rendering,
GitHub push or server submission was performed.

## Evidence

Run directory: `output/world_model_dataset/v0_2/c1_independent01/`.
Each of the two controllers was run in a fresh native process under free, resisted and
wall-backed overload conditions: six independent episodes, 240 Hz, 1.8 seconds each.
Original lane coordinates and fixture parameters were retained to isolate the packaging change.

All six actuator trajectories reproduce their original three-lane references exactly:
432 post-step states per episode, maximum position difference **0 m**, maximum velocity
difference **0 m/s**. Every pre-existing numerical summary also matches exactly, including
controller work and native contact impulse. This is evidence for these local rigid probes,
not a claim that other backends or soft bodies are bit-deterministic.

| Controller | Load | Command displacement (m) | Controller work (J) | Late saturation |
| --- | --- | ---: | ---: | ---: |
| Velocity effort | Free | 0.752424 | 0.331360 | 0% |
| Velocity effort | Resisted | 0.714226 | 0.854385 | 0% |
| Velocity effort | Wall-backed | 0.400000 | 0.330161 | 100% |
| Linear impedance | Free | 0.770405 | 0.064114 | 0% |
| Linear impedance | Resisted | 0.772069 | 1.298036 | 0% |
| Linear impedance | Wall-backed | 0.400001 | 2.068860 | 100% |

`reproduction_review.json` SHA-256:
`a3081855b546545b18b611366403dcf087b3717630923405cb0ddfd7b5c96b78`.

Direct cache review confirms a constant complete body set, zero fixed-body displacement,
normalized orientation quaternions, external force at most 8 N and zero force outside
the declared command interval. The minimum native contact separation across all runs is
−0.02182 mm. Each full-body trace contains t0 plus all 432 physics steps (433 records).

## Packaging and reproduction

Every independent directory contains:

- `episode.json`: structurally valid v0.2 draft manifest; lifecycle remains `draft` because
  observations and derived training labels are not yet produced;
- `resolved_config.json`, `initial_state.json`, `probe_initial.usda`: the exact selected
  scenario, complete formal initial state and resolved geometry/material/joint scene;
- `body_state_trace.jsonl`: pusher, anchor, floor and applicable load/wall at every physics
  step; angular velocity is converted from USD degrees/s to radians/s;
- command, actuator state and actuator effort traces; the bounded external input is not
  relabelled as total force or a measured joint reaction;
- `contacts.jsonl`: native contact point, normal, vector impulse and separation;
- final stage, numerical report, reproduction source snapshot and artifact hashes.

Effort trace `time_s` is the end of the physics interval: the listed force acted during
`[time_s - dt, time_s]`. Work is the signed discrete input-force × actuator-displacement
diagnostic, not an independently measured force sensor.

Native Isaac Python packages data without requiring `jsonschema`; structural review runs
on the WSL host. No automatic behavioral production gate was added. The reproduction
comparison reports entire-trajectory differences after each simulation.

Run all six into a **new** output directory:

```bash
python3 -m world_model_dataset.c1_reproduction \
  --output output/world_model_dataset/v0_2/c1_independent02
```

A single native run uses `causal_effort_probe.py --config <resolved_config.json>
--scenario <free|resisted|overload> --output <new-directory>`. Including `--scenario` enables
independent full-body episode packaging. The old combined probe remains available as a
capability reference; it is not observation-complete dataset output.

Tests: full suite 123 passed; focused packaging/contract suite 15 passed after the final
empty-trace comparison safeguard.

## Next work

1. Rotational actuator: free rotation, obstruction and torque saturation; finite authority
   and native angular state/effort/work must remain explicit.
2. C2 legal-initial-state no-action collision.
3. C2 finite-force rigid push.
4. C2 finite-impedance soft loading/compression.

C2 must additionally align control, complete physical state and two observation streams.
The six C1 load comparisons do not change the C2 completed-prototype count, which remains zero.
