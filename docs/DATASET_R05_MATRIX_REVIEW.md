# R05 support-removal stack-collapse matrix review

Reviewed 2026-09-16 directly from
`output/world_model_dataset/v0_1/m4_r05_physics_v02`. All 11 physics episodes passed
fixed-topology cache inspection, native per-point rigid contact export and exact one-shot
support removal. No videos were generated.

The accepted fixture places four bodies on a 0.6D pedestal with a 0.15D cumulative lean,
then disables the pedestal collision at 1.0 s. The floor is a common finite 30D × 30D plane.
An earlier 8D-wide floor allowed a low-friction block to leave the apparatus and fall out of
the world; that v01 result is preserved as fixture-boundary evidence and is not accepted.

All accepted initial stacks retain the three expected interbody edges immediately before
support removal. Maximum pre-action speed ranges from 0.001 to 0.085 m/s. A 0.20D lean and
a spherical top were rejected during manual review because they were already collapsing or
rolling before the action. The accepted lean boundary is 0.18D; the third top geometry is a
stable L shape. Superseded variants remain under the v02 `superseded...` directory.

| Case | Interbody edges after action | Maximum height drop | Final lateral/planar spread | Interpretation |
| --- | ---: | ---: | ---: | --- |
| Reference | 2 of 3 | 3.59D | 3.77D | top block separates; three-level remnant |
| Zero lean | 2 of 3 | 3.55D | 2.44D | symmetry is broken by impact, partial collapse |
| Lean boundary | 1 of 3 | 3.59D | 2.61D | stronger partial collapse |
| Low support | 0 of 3 | 3.19D | 4.88D | complete separation |
| High support | 3 of 3 | 0.83D | 0.33D | stack survives and remains in motion |
| Top low friction | 0 of 3 | 3.59D | 7.16D | widest complete scatter |
| Top light | 3 of 3 | 0.60D | 0.44D | stack survives |
| Top heavy | 2 new/changed edges | 3.59D | 2.74D | compact rearranged remnant |
| Top bouncy | 1 of 3 | 3.59D | 2.15D | partial collapse |
| Cylinder top | 0 of 3 | 3.47D | 3.76D | complete separation |
| L-shape top | 3 of 3 | 0.63D | 0.41D | tilted stack survives |

The support-height response is non-monotonic: a larger synchronized vertical drop can retain
the stack while a small drop tips it apart. It is kept as a contact-path bifurcation, not
reported as a monotonic height law. The high-support case still has 0.184 m/s maximum body
speed at the final sample, so its settling time is right-censored and its last pose is not
called a stable final structure.

Contact graphs are maintained from native FOUND/PERSIST/LOST events. Sleeping contacts are
retained even when PhysX stops emitting PERSIST callbacks. Because disabling a collider does
not reliably emit CONTACT_LOST, edges involving the explicitly disabled pedestal are removed
from the post-action graph by action semantics. This does not synthesize interbody contacts.
The matrix contains 107,469 native contact points, including 75,249 interbody points.

The worst native contact-point separation is -7.03 mm in the heavy-top case. This transient
stack-impact overlap is retained as diagnostic evidence. The matrix contains eight strict
single-variable pairs and three top geometries (rounded cube, cylinder and L shape). These
are physics-only development caches, not observation-complete release episodes.
