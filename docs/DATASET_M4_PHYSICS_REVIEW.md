# M4 canonical physics review

> **Causal-status note (2026-09-16):** this review accepts numerical physics and cache
> integrity under the historical v0.1 action semantics. It does not admit the trajectories
> to the causal v0.2 training corpus. The ten-event catalogue is now a regression suite;
> migration status is recorded in
> [the v0.2 causal migration audit](DATASET_V0_2_MIGRATION_AUDIT.md).

Reviewed 2026-09-16 from accepted immutable caches. The canonical physics implementation for
all ten core events is now present. This closes M4 physics development, not Dataset v0.1
publication: camera observations, environment deployment and final train/validation/test
splits remain M5 work.

| Event | Accepted matrix | Episodes | Strict pairs | Main reviewed distinction |
| --- | --- | ---: | ---: | --- |
| R01 | `m3_physics_v01` R01 subset | 20 | 12 | slope, friction and rolling/sliding geometry |
| R02 | `m4_r02_physics_v02` | 11 | 8 | successive stair contacts, bounce and intermediate rest |
| R03 | `m4_r03_physics_v02` | 11 | 8 | hit/miss, mass and restitution transfer |
| R04 | `m4_r04_physics_v02` | 11 | 8 | obstacle branch, clean miss and wedging |
| R05 | `m4_r05_physics_v02` | 11 | 8 | support-removal graph change and collapse bifurcation |
| V01 | `m4_v01_physics_v02` | 11 | 8 | drop height and modulus response |
| V02 | `m3_v02_material_corrected_v01` | 9 | 6 | corrected 10/25/40% compression × 30/100/300 kPa |
| V03 | `m4_v03_physics_v01` | 11 | 7 | load mass/modulus, unloading and offset |
| V04 | `m4_v04_physics_v01` | 11 | 8 | aperture width, pass/lodge, rate and geometry |
| V05 | `m4_v05_physics_v01` | 11 | 8 | rigid-soft hit/miss, impact speed, mass and modulus |

The accepted core comprises 117 physics episodes and 81 strict single-variable pairs. A
separate four-episode real-rigid supplement (`m4_real_rigid_physics_v02`) verifies STL assets
in R02/R03/R04/R05, bringing reviewed M4/M3 physical evidence used by this milestone to 121
episodes. Superseded smoke tests and rejected v01 fixture cases are not counted.

Across the volumetric events, declared Young's modulus, Poisson ratio and friction are checked
against native tensor readback. Fixed visible and simulation-Tet topology is preserved, and
Tet inversion is checked at every physics step. Soft per-contact impulse remains explicitly
`unavailable`; those episodes must be excluded from impulse-supervision tasks. Across rigid
events, native contact points, normals and impulses are retained.

Important interpretation limits remain:

- Right-censored trajectories are valid interaction observations but not stable final-state
  labels. This applies to known moving endpoints such as the R04 sphere miss, the R05
  high-support stack, and V04 sphere/capsule traversal.
- Geometric fixture overlap is a diagnostic and must not be presented as a native contact
  report or force measurement.
- Real STL rigid assets currently use convex-hull proxies. Exact concave collision remains a
  separate geometry capability.
- V02's corrected matrix intentionally repairs material validity on the rounded cube; the
  plan still forbids reopening an unbounded V02 sweep during this milestone.
- These caches do not yet satisfy observation-complete episode publication.

Manual cache review, not pre-run behavioral gates, determined whether the intended event
actually occurred. In particular it rejected a numerically valid chair run that never reached
the obstacle and led to a corrected action. Automated checks remain limited to data integrity,
native readback and obvious numerical failure.

A final unified-loader sweep opened all 121 accepted caches, validated each archived manifest,
loaded one state and geometry frame, and exercised the representation-appropriate contact API.
It read 90 rigid or rigid/volumetric native contact streams and confirmed explicit soft-contact
`unavailable` behavior for 31 pure-volume episodes. Three immutable early R01 caches retain a
legacy failure flag for the former 3 mm visual-surface/proxy overlap threshold; the loader accepts
them only when the matrix-level manual review says R01 is accepted and that diagnostic is the
sole failed check. No other failed check is bypassed.
