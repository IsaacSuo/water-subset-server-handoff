# R04 kinematic obstacle-push matrix review

Reviewed 2026-09-15 directly from
`output/world_model_dataset/v0_1/m4_r04_physics_v02`. All 11 physics episodes passed
cache integrity, fixed topology, native rigid-contact export and native kinematic-pusher
tracking. No videos were generated.

The first matrix revision exposed a fixture-design error during manual review: the nominal
wide-miss sphere correctly avoided the obstacle, but then rolled beyond the short finite
floor and fell. That result is preserved under the v01 `superseded_wide_miss...` directory.
The accepted v02 matrix extends the otherwise identical finite lane and moves the miss to
-1.35D. All entries use the same extended fixture; this preserves strict single-variable
counterfactuals instead of shortening only the miss episode.

| Episode family | Non-floor contact order | Forward progress | Final lateral displacement | Outcome |
| --- | --- | ---: | ---: | --- |
| Sphere reference | pusher → obstacle → left rail | 2.79D | -1.75D | left route |
| Sphere low friction | pusher → obstacle → left rail | 3.37D | -1.75D | left route, farther travel |
| Sphere slow push | pusher → obstacle → left rail | 2.43D | -1.75D | stuck boundary |
| Sphere fast push | pusher → obstacle → left rail | 1.76D | -1.74D | stuck boundary |
| Sphere obstacle +0.3D | pusher → obstacle → left rail | 7.62D | -1.75D | forward/left clear |
| Sphere mirrored start | pusher → obstacle → right rail | 2.79D | +1.75D | mirrored right route |
| Sphere wide miss | pusher only | 19.61D | approximately 0D | clean obstacle miss |
| Rounded cube, two frictions | pusher → obstacle | 1.44–1.45D | within 0.02D | wedged/stuck |
| Capsule, two frictions | pusher → obstacle | 1.74–1.84D | within 0.04D | wedged/stuck |

The matrix therefore records actual path branching rather than only scalar displacement:
the sign-reversed starting offset mirrors the rail branch; the wide offset removes obstacle
contact; and sphere, rounded-cube and capsule geometries separate routing from wedging. The
push-duration response is deliberately non-monotonic: equal push distance over 0.8, 0.5 and
0.3 seconds produces maximum speeds of 0.562, 0.901 and 1.501 m/s, but only the reference
timing clears. This is a valid contact-path bifurcation, not interpreted as a monotonic speed
benchmark.

The v02 wide miss remains on the finite floor for the full five seconds, with no obstacle
impulse and only pusher contact. It is still moving at 0.891 m/s at the final sample because
an ideal rigid sphere has no rolling-resistance model in this fixture; the episode measures
routing, not final rest. Rounded-cube and capsule visible surfaces reach 3.31–3.63 mm below
the nominal floor plane while wedged. This is retained as collision-proxy/contact-tolerance
evidence rather than hidden, and is below the current 20 mm numerical-integrity bound.

The ten non-miss v02 entries retain their v01 non-floor contact order and clear/stuck class.
The accepted matrix contains eight leaf-level single-variable pairs and three geometries.
These are physics-only development caches, not observation-complete release episodes.
