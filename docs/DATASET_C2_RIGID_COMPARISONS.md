# C2 rigid comparisons

2026-09-16. Four local physics-only runs, 2 s each at 240 Hz. Existing baselines
reused; direct state/contact/effort review, no additional rendering.

## Finite-force push

Same controller reference and scene; change either force authority or load mass.
Displacement is measured at 2 s. Lower force also delays first contact, so this
compares the whole commanded episode, not equal-duration contact intervals.

| Run | Force / load mass | Load displacement | Speed when command ends at 1.5 s | Final speed |
| --- | --- | --- | --- | --- |
| `c2_push_smoke01` (baseline) | 8 N / 0.5 kg | 0.830 m | 0.750 m/s | 0.420 m/s |
| `c2_push_force1_01` | 1 N / 0.5 kg | 0.390 m | 0.526 m/s | 0.194 m/s |
| `c2_push_load3kg_01` | 8 N / 3 kg | 0.348 m | 0.483 m/s | approximately 0 |

Weak-force contact starts at 1.10 s versus 0.7625 s for baseline/heavy load.
Neither variant reaches the guide stop. Force limits are respected; native
reported penetration stays below 0.004 mm. Weaker authority and a heavier load
both visibly reduce the resulting motion. Good enough to proceed.

## No-control collision

Initially one ball moves toward a stationary ball. Geometry and restitution
remain the same; change the incoming speed or only the target mass.

| Run | Incoming speed / target mass | Contact time | Incoming ball after collision | Target after collision |
| --- | --- | --- | --- | --- |
| `c2_none_smoke02` (baseline) | 1 m/s / 1 kg | 0.604 s | +0.300 m/s | +0.700 m/s |
| `c2_collision_speed125_01` | 1.25 m/s / 1 kg | 0.483 s | +0.375 m/s | +0.875 m/s |
| `c2_collision_target3kg_01` | 1 m/s / 3 kg | 0.604 s | −0.050 m/s | +0.350 m/s |

Velocities above are read four physics steps after first native interbody impulse.
Total horizontal momentum agrees before/after to within 6e-8 kg·m/s. Both balls
remain on the floor throughout each run. Increased incoming speed transfers more
motion; the heavier target causes a small rebound. No issue blocks further work.

Variant recipes live in `configs/dataset/v0_2/` with the corresponding names
(`c2_push_force1`, `c2_push_load3kg`, `c2_collision_speed125`,
`c2_collision_target3kg`). Results are under `output/world_model_dataset/v0_2/`.
Next: reuse these physical controls with real object geometry and scene placement.
