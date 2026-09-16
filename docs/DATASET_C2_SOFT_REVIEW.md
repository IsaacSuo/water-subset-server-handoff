# C2 soft compression experiments

Updated: 2026-09-16. Local only; no server submission.

## Setup

0.2 m soft cube on a floor, 1 kg dynamic plate on a world-anchored Z guide.
Gravity is enabled. Finite impedance: 6,000 N/m spring, 120 N·s/m damper.
The 5 s reference holds, compresses during 1–2 s, holds until 2.75 s, retracts
until 3.75 s, then observes recovery. References are not body pose writes.
PhysX GPU TGS: 240 Hz, 128 soft position iterations, cooking resolution 10
(1,331 nodes / 6,000 tets). Material is root-bound and checked by native readback.

`c2_soft_smoke02` completed the control/state/mesh-to-two-view path: 302 RGB-D and
segmentation frames. Later parameter comparisons use physics caches only.
The existing rigid push regression is unchanged. Source and raw data remain in
each output directory; this page keeps only the useful conclusions.

## Comparisons

Heights are measured relative to the 1 s pre-loading state under gravity, not
the undeformed bind pose. Each variant changes just one physical input.

| Run | Modulus / force limit | Before → minimum → end height | Interpretation |
| --- | --- | --- | --- |
| `c2_soft_smoke02` | 30 kPa / 250 N | 195.85 → 169.94 → 195.83 mm | 26 mm compression; no force saturation; almost full height recovery. |
| `c2_soft_stiffer01` | 100 kPa / 250 N | 199.16 → 188.06 → 198.92 mm | 11 mm compression; hold saturates at 250 N; recovery is sufficient to continue. |
| `c2_soft_force50_01` | 30 kPa / 50 N | 195.85 → 186.05 → 195.81 mm | 9.8 mm compression; hold saturates at 50 N; almost full height recovery. |

Both comparisons have zero inverted tets (minimum J: 0.931 / 0.909).
Modulus readback matches each recipe. A harder object demands more actuator force;
with weaker authority the same object compresses less. Effects are clear enough
to proceed; no extra repeat or rendering is needed for these development tests.

## Necessary caveats and next step

Soft point-contact impulse is **unavailable**; geometric contact is a derived
collision-node sampling diagnostic, not solver force/impulse. Native nodal
velocity exports can become stale when geometry stops updating: the initial
`c2_soft_smoke01` has this late issue, but its earlier compression data remain
useful. It is not evidence of exact live-velocity reproduction.

Development now directly reads height, actuator effort and obvious instability
after each run, leaves a short conclusion and proceeds. No repeated render,
per-frame hash review or exact-repeatability prerequisite for these comparisons.
Small jitter does not block development. Three prototype paths are available;
additional controlled comparisons come next, not dataset-scale production.
