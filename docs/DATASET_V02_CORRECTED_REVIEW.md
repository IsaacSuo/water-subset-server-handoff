# Corrected V02 physical review

Reviewed 2026-09-15 directly from `m3_v02_material_corrected_v01` native caches.
The nine-run matrix crosses 10/25/40% commanded compression with 30/100/300 kPa.
All runs use 240 Hz physics, 30 Hz capture, 5 s duration and 128 position iterations.

All nine native material tensor readbacks match declared modulus, Poisson ratio and
friction. Six modulus pairs have identical rest geometry and exactly one resolved physical
leaf changed. There are 10,800 checked physics steps, zero Tet inversions, and a minimum
Jacobian of 0.36709. Plate trajectory error is at most 1.22e-8 m.

| Commanded compression | Observed range | Maximum sampled collision-shell/fixture penetration |
| --- | --- | --- |
| 10% | 9.59–9.68% | 1.09 mm |
| 25% | 24.25–24.51% | 2.58 mm |
| 40% | 38.68–39.46% | 4.66 mm |

These penetration values remain approximation/error evidence; integrity pass is not a
claim of exact nonpenetrating contact. The 40% group is a deformation boundary case.
Soft per-contact force and impulse remain unavailable. No geometry-derived or estimated
impulse is substituted, and V02 is excluded from flexible-contact impulse tasks.

## Recovery correction

The old recovery label used the authored no-load rest mesh and started counting after
the plate had finished withdrawing. That conflated gravity-induced supported deformation
with failure to recover, and labeled recovery during withdrawal as zero elapsed time.

The supplemental `equilibrium_recovery_review.json` leaves original metrics/manifests
untouched. Its reference is the shared 1.0 s capture immediately before compression; it
removes whole-object rigid motion and counts from withdrawal onset at 2.5 s. It publishes
0.001D, 0.005D and 0.01D crossing times with a 0.5 s sustained interval as diagnostics,
not acceptance rules. At D=0.2 m, the 0.005D level is 1 mm.

All nine runs return within 1 mm in 0.133–0.467 s from withdrawal onset. Their final
shape difference from pre-action supported equilibrium is 0.014–0.118 mm. The soft
profile's approximately 2.1 mm residual against the authored rest mesh is already present
before compression and is not evidence of irreversible deformation. New audits retain
the authored-rest metric under an explicit legacy name and expose the corrected reference
and time origin.

The corrected matrix does not restore historical compression-speed comparisons: its
compression duration is fixed at 1 s. It also does not establish a monotonic modulus-
identification benchmark just because materials are now registered. Imposed displacement
mostly controls height; deformation/recovery, not unmeasured plate force, is the valid truth.

The 25% / 100 kPa repetition passed exact action replay and bounded GPU-state comparison.
Maximum instantaneous node-position difference is 2.883 mm (not bit-exact); compression
differs by 0.00189 percentage points and final authored-rest residual by 0.00037 mm.
Evidence is recorded separately in `corrected_replay_review.json`; its scope is this one
profile, not an all-material determinism claim. No videos were generated.
Physics-only caches are not finalized observation-complete episodes; v0.1 schema, release
hashes and historical caches remain frozen.
