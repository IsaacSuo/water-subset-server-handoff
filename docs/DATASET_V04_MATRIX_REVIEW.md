# V04 soft-body aperture matrix review

Reviewed 2026-09-16 directly from
`output/world_model_dataset/v0_1/m4_v04_physics_v01`. All 11 physics episodes passed
fixed-topology cache inspection, native material tensor readback, exact kinematic-pusher
tracking and all-physics-step Tet inversion checks. No videos were generated.

The first thin-wall smoke test was rejected: the pusher continued through a blocked body,
producing 469 inverted Tets and a 16.7 m/s terminal speed. A second smoke stopped the pusher
at the wall face and removed the large energy injection, but the zero-radius slit edge still
concentrated load into one collision-Tet row: 29 Tets inverted and the visible body expanded
laterally instead of squeezing. Neither smoke is accepted.

The accepted fixture has a short symmetric converging guide followed by a parallel throat.
The declared `aperture_width_D` is the throat width. The guide changes how strain is applied,
not the measured opening. Its inlet is 1.3D wide, the pusher is 0.6 times the throat width and
can traverse the aperture, and the finite pusher travel is part of the declared action.

| Case | Outcome | Maximum lateral compression | Minimum Tet J | Final shape residual | Final speed |
| --- | --- | ---: | ---: | ---: | ---: |
| Reference, 0.85D | passed | 11.1% | 0.608 | 0.180 mm | 0.0001 m/s |
| Narrow, 0.75D | passed | 17.8% | 0.338 | 0.164 mm | 0.0001 m/s |
| Wide, 1.00D | passed | 0.5% | 0.937 | 0.185 mm | 0.0001 m/s |
| Soft, 30 kPa | passed | 11.6% | 0.609 | 0.684 mm | 0.0004 m/s |
| Stiff, 300 kPa | passed | 10.9% | 0.576 | 0.083 mm | 0.0004 m/s |
| Fast push, 2.75 s | passed | 11.1% | 0.614 | 0.195 mm | 0.0001 m/s |
| Slow push, 4.00 s | passed | 11.1% | 0.618 | 0.182 mm | 0.0002 m/s |
| Short push, 2.9D | partially through | 11.1% | 0.592 | 8.103 mm | 0.0003 m/s |
| Long push, 3.7D | passed | 10.7% | 0.590 | 0.182 mm | 0.0002 m/s |
| Sphere | passed | 14.8% | 0.575 | 1.423 mm | 0.1219 m/s |
| Capsule | passed | 14.3% | 0.577 | 0.238 mm | 0.0264 m/s |

The width response is physically legible: the 1.00D opening is effectively clearance, while
0.85D and 0.75D openings require progressively larger lateral compression. The short-push
episode is a useful boundary result: the body remains stably lodged in the throat, hence its
8.1 mm residual is constrained deformation rather than failed recovery.

The sphere and capsule have completely passed but are still rolling/sliding at the end of the
6.5 s observation. Their trajectories are explicitly treated as right-censored for final rest;
their last poses are not stable final-state labels. Future V04 audits emit
`motion_settled_at_end` and `observation_right_censored`; this review supplies the same meaning
for the immutable v01 caches.

Visible-surface overlap with the analytical fixture peaks at 4.53 mm in the narrow case;
collision-shell overlap peaks at 2.77 mm. These are published geometric diagnostics, not
solver contact impulses. Soft contact impulse remains `unavailable`; no zero or estimated
values are substituted. The matrix contains eight strict single-variable pairs and three
geometries (rounded cube, sphere and capsule). These are physics-only development caches,
not observation-complete release episodes.
