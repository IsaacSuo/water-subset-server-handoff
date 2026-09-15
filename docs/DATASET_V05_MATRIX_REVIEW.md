# V05 rigid/soft impact matrix review

Reviewed 2026-09-15 directly from
`output/world_model_dataset/v0_1/m4_v05_physics_v01`. All 11 physics episodes passed
cache integrity, shared-time-axis, native material readback and every-step Tet checks.
No videos were generated.

The matrix contains eight strict single-variable pairs around a sphere/soft-cube baseline:
low/high impact speed, light/heavy projectile, soft/stiff target, eccentric impact and a
lateral miss. Capsule-projectile and soft-sphere-target cases add standalone geometry
coverage; they are not mislabeled as leaf-level counterfactual pairs.

| Family | Low / soft / light | Reference | High / stiff / heavy |
| --- | ---: | ---: | ---: |
| Impact speed: target max nonrigid RMS | 1.759 mm | 3.563 mm | 7.166 mm |
| Impact speed: target max COM displacement | 0.882 m | 1.764 m | 3.450 m |
| Projectile mass: target max nonrigid RMS | 2.588 mm | 3.563 mm | 4.552 mm |
| Projectile mass: target max COM displacement | 1.029 m | 1.764 m | 2.685 m |
| Target modulus: target max nonrigid RMS | 7.479 mm | 3.563 mm | 1.735 mm |
| Target modulus: max local node displacement | 33.17 mm | 21.11 mm | 13.64 mm |

The miss retains 1.5 m/s exactly, has no sampled target contact, and moves the target only
2.05 mm at most from settling. The centered baseline projectile changes from 1.5 m/s to a
small reverse velocity of about 0.054 m/s. The eccentric hit remains coupled and produces
3.650 mm peak nonrigid RMS. Capsule/cube and sphere/sphere both contact and deform without
inversion.

Across all 7,920 checked physics steps there are zero Tet inversions. The worst minimum
Jacobian is 0.5892 in the 3 m/s speed boundary case. Maximum sampled penetration lower
bound is also in that case at 1.959 mm. The capsule diagnostic uses convex support planes,
not an incorrect bounding sphere; its sampled penetration is 0.957 mm. Local Tet-edge
shortening, nonrigid RMS and maximum local node displacement are all reported because
whole-object axis extent can hide a localized impact indentation.

Interpretation limits remain unchanged: the free target deliberately slides on a zero-
friction support, so long COM travel measures transferred motion rather than a realistic
stopping distance. Collision-node penetration is sampled approximation evidence. Native
callback impulses cover rigid/fixture contacts only; rigid/soft per-contact force and
impulse remain unavailable and are never estimated from velocity changes. These are
physics-only development caches, not observation-complete release episodes.
