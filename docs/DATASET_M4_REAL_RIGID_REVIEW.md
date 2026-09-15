# M4 real-rigid geometry supplement review

Reviewed 2026-09-16 directly from
`output/world_model_dataset/v0_1/m4_real_rigid_physics_v02`. Four independent real-asset
episodes passed cache and native rigid-contact review. No videos were generated.

The supplement adds one existing project STL to each rigid event family that needed a
non-diagnostic shape:

| Event | Asset | Reviewed behavior |
| --- | --- | --- |
| R03 | banana | interbody contact at 0.804 s; 44 points; 0.292 s contact interval |
| R02 | carrot | contacts steps 0, 1 and 2 in order, tumbles by about 180°, then rests on step 2 |
| R04 | chair | pusher then obstacle contact; stable `stuck_behind_obstacle` outcome |
| R05 | elephant top | support removal drops the stack 0.61D; all three interbody edges survive |

The first chair attempt used a 0.5 s push. It passed numerical checks but stopped before the
obstacle, so manual review rejected it as an event-level false positive. The accepted v02
uses the same 1.5D displacement over 0.3 s, records pusher then obstacle contact and settles
behind the obstacle. This is why physical acceptance still includes direct post-run review
instead of equating schema validity with a meaningful interaction.

Each registry entry stores the repository-relative STL path and expected SHA-256. Geometry
preparation refuses a source-hash mismatch, normalizes maximum extent to D=0.2 m, derives
mass and inertia from the closed mesh and uses a convex-hull rigid collision proxy. The proxy
does not preserve chair cavities or other concavities; these episodes validate real-shape
pipeline compatibility, not exact concave contact. They are independent geometry baselines,
not leaf-level counterfactual pairs.
