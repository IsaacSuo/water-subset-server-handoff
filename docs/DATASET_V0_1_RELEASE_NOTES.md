# Dataset Contract v0.1 release scope

Frozen 2026-09-14 after the R01/V02 vertical prototypes. Release hashes are recorded in
`configs/dataset/contract_v0_1_release.json`.

## Erratum — 2026-09-15

The Dataset Contract v0.1 schema and release hashes remain frozen. A later native tensor
readback added during M4 found that the original V02 material prim was not registered by the
PhysX deformable-material backend. All 15 historical V02 runs therefore used the backend
default material despite declaring 30/100/300 kPa profiles.

Consequently, the old V02 caches remain accepted only as evidence for action execution,
fixed-topology surface/Tet export, nodal state export, per-step inversion checks, and stable
compression under one unspecified backend-default material. Acceptance of their declared
material comparisons, and all earlier conclusions about modulus sensitivity, is withdrawn.
M3 physical scope is not closed again until a corrected V02 matrix records matching native
material tensor readback. Historical caches and manifests are retained rather than rewritten.

## Accepted scope

- R01 rigid ramp release: native pose, velocity, angular velocity and per-point PhysX
  contact position, normal and impulse. The 20-episode physics matrix is accepted.
- V02 volume-deformable plate compression: visible surface, simulation Tet state, native
  nodal velocity, compression, volume, recovery, penetration diagnostics and per-substep
  inversion evidence. Subject to the erratum above, the 15-episode matrix is accepted only
  for representation/action/topology stability under the backend default material.
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
- The historical V02 displacement-controlled modulus and compression-speed comparison is
  invalid because its declared materials were not active in the solver. A corrected matrix
  is required before judging whether this event is useful for material identification.

## Local review evidence

- `output/world_model_dataset/v0_1/m3_physics_v01/manual_physics_review.json`
- `output/world_model_dataset/v0_1/m3_physics_v01/physics_consistency_review.json`
- `output/world_model_dataset/v0_1/v02_rapid_release_review.json`

These outputs are development evidence, not repository release artifacts.
