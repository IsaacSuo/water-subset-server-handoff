# C2 finite-force rigid push — first smoke

Updated: 2026-09-16. Local only; no server submission or GitHub push.

## Scope and implementation

`output/world_model_dataset/v0_2/c2_push_smoke01` is a manually reviewed, complete
single smoke candidate, not a completed C2 prototype matrix or a dataset release.
The no-control collision and finite-force push now have complete state-to-observation
paths. Soft finite-impedance loading and all three paired matrices remain pending.

The 1 kg dynamic pusher slides along a finite X prismatic joint anchored to the world,
not to an invisible kinematic actor. Gravity remains enabled for both rigid bodies;
the guide carries the pusher's vertical reaction. A 0.5 kg cube rests on a physical
floor with static/dynamic friction 0.3/0.2 and zero restitution. All three bodies
remain present, colliding and visible throughout. The pusher is 0.2 × 0.5 × 0.3 m;
the cube is 0.3 m per side. Guide travel limits are 0–1.5 m relative to t0.

From 0.2 to 1.5 s, velocity feedback requests `20 * (0.8 - measured_vx)` N,
clipped to ±8 N. Outside that interval the external force is zero, **not** a
braking servo or a frozen pose. Target velocity is a force-law input, never a solver
state assignment. Initial velocity/mass/inertia are checked with native tensor
readback before formal t0. After t0 only bounded actuator force attributes change;
all body motion is integrated by PhysX. No subject state write or participant toggle.

Backend: Isaac Sim 6.0.1 / PhysX 110.1.13, CPU TGS, 240 Hz, position/velocity
iterations 16/4, contact/rest offset 2 mm/0, speculative CCD disabled. These are
fixed backend state data, not a claim of calibrated or converged real-world physics.

## Direct numerical review

- 480 physical steps, 481 complete body and actuator samples, 480 effort intervals.
- First pusher–cube contact: 0.7625 s; 1,192 native contact-point records for that
  pair, 2,772 records including floor support.
- Pusher reaches 0.79985 m/s before contact. Under load both move approximately
  0.7498 m/s at command end, with a genuine finite-force tracking error.
- At 2 s pusher/cube velocities are 0.41998/0.41990 m/s. The cube advances
  0.83042 m. Neither is settled; the horizon is explicitly right-censored.
- Peak external force 8 N; inactive force exactly zero; active saturation 3.846%.
  Derived signed external `F*dx` work is 1.08605 J. This is not contact work or
  contact-force supervision. Native contact impulses are separately preserved.
- Worst reported separation −2.444 µm; maximum body speed below 0.800 m/s.
  Pusher guide height drift is about 7.15 nm; final travel 1.23042 m stays below
  the 1.5 m upper stop. No stop constraint confounds this normal-load run.

`c2_push_replay01` reconstructs the scene in an independent native process. Full
body states, contact reports, declared commands, actuator states and actuator
efforts are byte-for-byte identical. Its observations are intentionally not repeated.

## Two-view observations and reader

Cache-only RTX produces 61 frames per fixed camera (front/rear), 640 × 480, 30 Hz,
including t0 and the 2 s endpoint: 122 RGB-D/instance-segmentation frames total.
All 244 projected pusher/cube centres agree with the declared instance IDs. Across
5,206 interior floor ray samples at t=0, 0.8 and 2 s, median/max optical-depth errors
are 0.566/2.625 µm. RGB/NPZ hashes, frame timestamps and all actuator/body state
timestamps were checked directly; no video was produced. Camera metadata already
uses integer pixel centres with principal point (319.5, 239.5).

The loader now exposes `controls()`, `actuator_states()` and `actuator_efforts()`
alongside states, native contacts, resolved input profiles and observations. The
finalizer derives actual actuator–subject contact runs, subject displacement and
bounded external-input work diagnostics. It does not label a desired outcome as
an observed interaction, and does not interpolate contact across missing-step gaps.

## Reproduction and limitations

```bash
python3 -m world_model_dataset.causal_runner \
  --config configs/dataset/v0_2/c2_rigid_push.json \
  --output output/world_model_dataset/v0_2/<new_unique_id>
# Inspect native states, contact reports, initialization readback and effort trace.
python3 -m world_model_dataset.causal_observe \
  --episode output/world_model_dataset/v0_2/<new_unique_id>
# Inspect synchronized sensor arrays and record an explicit human_review.json.
python3 -m world_model_dataset.causal_finalize \
  --episode output/world_model_dataset/v0_2/<new_unique_id>
```

During the first run, Windows exited normally after complete native export but the
WSL launch wrapper remained waiting. Complete native outputs were independently
checked and packaged, and only the stale local wrapper was terminated. The early
observation attempt failed for missing packaging, before generating frames; retry
used the same physics cache successfully. Both logs are retained. The independent
repetition completed through the normal runner, so this is recorded as a launcher
incident, not silently converted into a physics failure or a general timeout fix.

The current rigid backend deliberately supports only one translational effort
controller/command with a world-anchored X slider; rotary C1 capability is not
claimed as a completed C2 production backend. Normal, blocked and boundary cases,
at least six strict single-variable pairs, broader geometry/material coverage and
the soft prototype are still required before C2 closes.
