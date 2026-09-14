# Dataset Contract v0.1 release scope

Frozen 2026-09-14 after the R01/V02 vertical prototypes. Release hashes are recorded in
`configs/dataset/contract_v0_1_release.json`.

## Accepted scope

- R01 rigid ramp release: native pose, velocity, angular velocity and per-point PhysX
  contact position, normal and impulse. The 20-episode physics matrix is accepted.
- V02 volume-deformable plate compression: visible surface, simulation Tet state, native
  nodal velocity, compression, volume, recovery, penetration diagnostics and per-substep
  inversion evidence. The 15-episode matrix is accepted for these tasks.
- Physics and observation generation are separate. Development can review a physics-only
  cache; a published completed episode still requires its declared observations and final
  manifest. Rendering an accepted cache never reruns PhysX.

## Explicit exclusions

- V02 does not provide per-contact soft impulse or force. The current public backend path
  reports no reliable value; the capability is `unavailable`, never an empty, zero or
  estimated substitute. V02 episodes are excluded from contact-impulse tasks.
- Geometric overlap/intersection is a derived diagnostic, not a solver contact report.
- GPU deformable trajectories are not claimed bit-deterministic. Aggregate compression,
  recovery, volume and topology results are repeatable; individual transient nodes can differ.
- The current V02 displacement-controlled modulus and compression-speed families are too
  weak for a material-identification benchmark. A 0.1 s withdrawal probe activates free
  oscillation but does not yield a robust monotonic 30 Hz modulus label. Material inference
  must use a later high-rate free-response or force-observable event.

## Local review evidence

- `output/world_model_dataset/v0_1/m3_physics_v01/manual_physics_review.json`
- `output/world_model_dataset/v0_1/m3_physics_v01/physics_consistency_review.json`
- `output/world_model_dataset/v0_1/v02_rapid_release_review.json`

These outputs are development evidence, not repository release artifacts.
