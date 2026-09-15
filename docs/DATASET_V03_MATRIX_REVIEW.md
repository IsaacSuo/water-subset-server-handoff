# V03 rigid-load removal matrix review

Reviewed 2026-09-15 directly from
`output/world_model_dataset/v0_1/m4_v03_physics_v01`. All 11 physics episodes passed
shared-cache integrity, native material readback, action execution and every-step Tet
inspection. No videos were generated.

The target settles alone until 0.5 s. A rigid load held 0.02D above it is then switched from
kinematic to dynamic, so loading is caused by native gravity and rigid/deformable contact.
After the declared duration the rigid body and its collision are deactivated. The action is
therefore not a displacement-controlled V02 plate under another name.

| Counterfactual | Low / soft | Reference | High / stiff |
| --- | ---: | ---: | ---: |
| Load mass: peak height compression | 1.85% | 4.17% | 7.89% |
| Load mass: pre-removal height compression | 0.62% | 1.44% | 2.67% |
| Load mass: pre-removal nonrigid RMS | 0.619 mm | 1.222 mm | 2.441 mm |
| Target modulus: peak height compression | 12.42% | 4.17% | 1.20% |
| Target modulus: pre-removal height compression | 3.77% | 1.44% | 0.27% |
| Target modulus: pre-removal nonrigid RMS | 3.903 mm | 1.222 mm | 0.420 mm |

The 1.0, 1.5 and 2.5 s load-duration variants reach nearly the same loaded state: their
pre-removal nonrigid RMS values are 1.245, 1.222 and 1.231 mm. This is expected for the
current damped elastic material. Duration changes the removal action time, but this matrix
does not demonstrate physical creep, permanent set or a monotonic duration effect. Those
claims require a validated viscoelastic/plastic representation.

Geometry changes produce qualitatively different load paths. The cylinder stays centered
and produces 0.724 mm pre-removal nonrigid RMS. A spherical load remains in contact and
causes localized deformation even though whole-surface height compression returns to zero;
its substep local displacement reaches 14.30 mm, showing why height alone is not a sufficient
metric. The eccentric cube and the cube on a soft spherical target slide away at 1.225 s and
1.288 s, before the 2.0 s removal command. They are retained as loss-of-contact geometry
branches, not mislabeled as sustained loading.

All 13,200 physics steps across the matrix retain positive Tet orientation. The worst minimum
Jacobian is 0.6996 for the soft-sphere target. The largest sampled rigid/soft penetration
lower bound is 3.144 mm for the soft material. The soft-sphere target remains above the
strict 0.5%D recovery band at the end (1.380 mm residual); this is right-censored recovery,
not asserted permanent deformation.

The matrix has seven strict leaf-level pairs plus three standalone geometry cases. Native
rigid/deformable per-contact force and impulse remain unavailable through the public callback;
no zero or velocity-derived impulse is substituted. The rigid actor deactivation time is
recorded explicitly in the native report and action stream. Multi-object observation replay
must use that action to suppress the removed load; the current single-object development
renderer is not used for these caches.
