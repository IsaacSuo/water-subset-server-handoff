# V05 mixed-backend smoke review

Reviewed 2026-09-15 from native caches/code. No video or physics replay required.

Accepted mechanics controls, relative to `output/world_model_dataset/v0_1/`:

- Hit: `v05_sphere_soft_cube_impact_smoke04`.
- Same-speed lateral miss: `v05_sphere_soft_cube_miss_smoke01`.

Both use a 0.2 m sphere projectile and rounded-cube soft target, 100 kPa reference
material, launch at 0.5 s with 1.5 m/s, 240 Hz physics, 60 Hz captures, 3 s duration,
and 128 deformable position iterations. Their sole semantic change is
`/fixture_parameters/impact_offset_D`: 0 versus 1.25. Supporting boxes and action
files are identical. Initial rest geometry and material references are identical.

| Measurement | Hit | Miss |
| --- | --- | --- |
| Projectile comparison speed, before → after | 1.5 → 0.05378 m/s (small reverse X component) | 1.5 → 1.5 m/s |
| Maximum target COM displacement | 1.76418 m | 0.00205 m |
| Every-step maximum local nonrigid displacement | 21.11 mm | 2.47 mm |
| Every-step maximum nonrigid RMS | 3.563 mm | 1.096 mm |
| Minimum Tet Jacobian | 0.70104 | 0.97000 |
| Inverted Tets across 720 steps | 0 | 0 |
| Geometric contact interval | 0.63333–0.69167 s | None; minimum sampled gap 49.47 mm |
| Sampled penetration lower bound | 0.242 mm | 0 |
| Final surface shape residual | 0.673 mm | 0.672 mm |

Native material tensor readback matches E=100000 Pa, nu=0.35, dynamic friction=0.3
in both runs. Native velocities come from PhysX USD velocity attributes after explicit
simulate/fetch, with tensor fallback; no position difference is used. Each capture holds
both objects on one timestamp/step. `cache_sync_preview.png` in the hit directory shows
the shared 0.650 s / step 156 geometry from two diagnostic views. Arrays and COM positions
were also checked, so the preview is not the sole synchronization evidence.

Interpretation: impact transfers motion into target translation and local deformation;
miss isolates gravity/settling deformation. The nearly equal final residuals describe
supported equilibrium, not a persistent impact scar. Whole-object X extent compression
is almost zero despite local deformation and must not be advertised as local strain.
The free target slides far because the diagnostic support has zero friction; this smoke
tests coupling, not realistic stopping distance.

Scope limits: sphere/collision-node penetration is a sampled lower bound, not exact solver
penetration. Native contact callbacks only identify projectile/floor in these controls.
Rigid/soft contact force and impulse are unavailable; velocity change is not substituted
as per-contact impulse. These runs do not qualify for flexible-contact impulse tasks.
They are physics-only prepared caches, not finalized observation-complete episodes.

Earlier hit `smoke03` is retained as the shorter-floor mechanics diagnostic. `smoke01/02`
failed during initialization/readback and are not accepted physics results. V05 formal
matrix and capsule-specific collision diagnostics remain to implement after corrected V02.
