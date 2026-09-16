# C2 real objects and first environment layout

2026-09-16. Local development only; no server submission or video rendering.

Two existing STL assets now use the causal finite-force pusher. Both have maximum
extent 0.3 m, mass 0.5 kg and an 8 N actuator limit. Each run lasts 2 s at 240 Hz,
with 481 body states, 480 effort records and native rigid contact reports. Cached
visual geometry is centered at its mass centroid; full inertia and principal axes
are authored explicitly. Native inertia readbacks differ by less than 6e-10 kg m².
Initial height is calculated from the rotated mesh, on the canonical z=0 support.

| Current run | Collision representation | Forward displacement | Peak speed | Deepest visual point below floor |
| --- | --- | --- | --- | --- |
| `c2_push_banana03` | Convex decomposition, CPU | 0.822 m | 0.751 m/s | 0.0006 mm |
| `c2_push_elephant04` | SDF 384, GPU | 0.816 m | 0.758 m/s | 0.554 mm transient; approximately zero at end |

Both respect 8 N and retain solver-only state evolution after t0. These are
geometry coverage examples, **not** a strict single-variable pair: appearance,
support height, collision representation and CPU/GPU numerics differ.
Elephant changes orientation by 15.9 degrees before pusher contact; its 24.4-degree
final rotation includes gravitational settling, not only the effect of pushing.

## Collision correction

The first single-convex-hull runs (`banana01`, `elephant01`, with prefix `c2_push_`)
put visual vertices 4.1/10.9 mm below the floor. Increasing the hull vertex limit
to 255 (`02`) did not change either trajectory. Reading the actual cooked elephant
hull showed only 34 vertices: its surface stayed on the floor while the visual
mesh protruded. This was collider approximation error, not deep solver penetration.

Convex decomposition fixed banana. Its previous 130-degree flip disappears; do
not interpret that diagnostic flip as trustworthy geometry-dependent behavior.
Elephant decomposition (`elephant03`) still had 11.4 mm visual penetration, so the
current recipe uses the existing multi-object project's elephant SDF policy.
The earlier runs remain diagnostic caches, not current representative results.
`inspect_causal_collision.py` reads cooked hulls without advancing physics.

## Warehouse placement

`configs/dataset/v0_2/layouts/warehouse_push.json` reuses the known warehouse support
point. The scene's Y-up coordinates are converted to the episode's Z-up frame;
the existing 5 cm support slab sits on the warehouse floor. Bodies and the two
cameras retain their cache coordinates. The exported initial layout is:

`output/world_model_dataset/v0_2/c2_push_banana03/layouts/warehouse_push/scene.usda`

This is a **layout-only composition**, not warehouse-native collision simulation
or a completed observation episode. It excludes background physics APIs and
imports no background PhysicsScene. No RGB/depth frames or video were rendered.

## Reuse

Run `python3 -m world_model_dataset.causal_runner --config
configs/dataset/v0_2/c2_push_banana.json --output <new-output-directory>` from WSL;
substitute `c2_push_elephant.json` for the SDF example. The launcher uses local
Windows Isaac Sim. For the initial warehouse layout, run `causal_render.py` through
Isaac's Python with `--episode <episode> --layout-only --environment-layout
configs/dataset/v0_2/layouts/warehouse_push.json`. The environment path is local.

Eight focused episode tests pass, including rotated STL placement, independent
geometry profiles and explicit mass/inertia preparation. The two completed native
runs load through the existing causal loader. Next: extend real-object and
environment coverage with the existing controls, without another broad parameter
sweep. Environment observations remain separate from physical simulation.
