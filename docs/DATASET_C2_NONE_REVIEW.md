# C2 no-control collision: first complete smoke

Completed locally on 2026-09-16. One no-control collision smoke now has complete native
state/contact, two-view RGB-D/segmentation, derived annotations/outcomes and a readable
v0.2 candidate manifest. **This is not acceptance of the whole prototype matrix or C2.**
No Blender render, video, GitHub push or server job was produced.

## Physical setup and result

`configs/dataset/v0_2/c2_none_collision.json` resolves a low-friction support floor and
two 0.1 m-radius, 1 kg spheres. The left sphere starts at 1 m/s and the right at rest.
The initial surfaces do not intersect; floor support contact is valid. Gravity is active,
object restitution is 0.4, and all later state updates belong to PhysX. There are no rails,
gate removal, actor switches, post-t0 velocity writes or hidden support geometry.

The accepted smoke is `output/world_model_dataset/v0_2/c2_none_smoke02/`:

- 2 seconds at 240 Hz: 480 explicit physics steps, 481 full-body records including t0.
- Native tensor t0 readback: velocities 1 and 0 m/s, masses both 1 kg, diagonal inertias
  approximately 0.004 kg·m². These are solver readbacks, not merely authored USD values.
- First interbody contact at 0.604167 s. Post-impact horizontal speeds are 0.29999998 and
  0.69999999 m/s, matching the analytic 0.3/0.7 result for this diagnostic pair.
- Horizontal momentum is 1 → 0.99999997 kg·m/s and horizontal kinetic-energy ratio is
  0.57999997. These are state-derived diagnostics, not calibrated real-world truth.
- Both subjects still move at 2 s; the stored final outcome is explicitly right-censored.

An independent reconstruction from the same config,
`output/world_model_dataset/v0_2/c2_none_replay01/`, reproduces every state field exactly,
and all 917 native point-contact records match exactly. Its observations were not rerendered.
This local CPU rigid result does not imply bit-determinism for soft bodies or other backends.

Full-state SHA-256 for both runs:
`c28cf27e31f5567cadd956e157a58c80fc0055bb2f977f47cb66338be2ed3238`.

## Initialization failure retained separately

`c2_none_smoke01` is rejected. The first implementation authored USD velocity before solver
load, but that path did not initialize the solver velocity in this manual simulation workflow.
It incorrectly captured authored 1 m/s at t0 and then native zero on step 1; no collision occurred.

The corrected backend loads physics, sets velocities through the rigid-body tensor API in
the **pre-t0 preparation phase**, reads them back, then captures formal t0. The simulation loop
only simulates and reads state. The rejected cache and report remain unchanged, with a separate
review recording the failure; they are not admitted as a valid natural-evolution episode.

## Two-view observation review

The accepted cache was replayed in a fresh, physics-free RTX stage. Both spheres and the floor
are present continuously. Two fixed cameras (`front`, `rear`) each produce 61 frames at 30 Hz,
640 × 480, from t0 through 2 s: 122 RGB/depth/segmentation observations in total.

Review checked all frame hashes, timestamps and physics-step references. Projection of each
native subject centre gives its correct stable instance label in both views at every frame
(244 checks, zero mismatches), including after impact. RGB is decodable and aligns in shape
with depth and segmentation.

The first sensor export's intrinsics used edge-origin pixels. Ground-plane depth verification
established integer-centre intrinsics with principal point `(319.5, 239.5)`. The raw index is
preserved; `observations/index.calibrated.json` applies the −0.5 pixel convention correction
and records its source hash. Native pixels were not changed or rerendered. Among 5,586 interior
floor ray samples across both views and three times, median depth error is 0.56 µm and maximum
2.46 µm. Floor side faces and boundaries are not compared against the top plane.

RTX tessellates implicit spheres, whereas PhysX uses exact sphere collision. Sampled central
rays have about 1.7–4.1 mm visible-depth difference from the analytic sphere. This is disclosed
as renderer geometry approximation, not camera/time misalignment or exact collider-depth truth.
This smoke also retains the backend's tiny support-contact velocity/position discrepancies;
no continuous-physics convergence or calibrated physical-response claim is made.

The complete local candidate occupies about 20 MB. No video was encoded.

## Interface and reproduction

New modules are separate from frozen v0.1 execution:

- `causal_runner.py`: config resolution, immutable preparation, native invocation and physical
  record packaging; backend currently supports joint-free rigid `none` only.
- `native_causal_rigid.py`: native rigid creation, tensor initialization/readback, per-step state
  and native point impulses.
- `causal_render.py` / `causal_observe.py`: all-body two-view cache replay and sensor packaging.
- `causal_finalize.py`: derived smoke annotations/outcomes after explicit human acceptance.
- `causal_loader.py`: v0.2 reader and version-aware `open_episode` bridge to the old loader.

The optional draft `system.resolved_inputs` artifact stores resolved geometry, physics,
appearance, numerics and cameras. C1/v0.2 drafts without it remain readable; they retain their
initial USD reproduction snapshot. Frozen v0.1 schemas and historical caches are not rewritten.

```bash
python3 -m world_model_dataset.causal_runner \
  --config configs/dataset/v0_2/c2_none_collision.json \
  --output output/world_model_dataset/v0_2/c2_none_next

# Review native state/contact first; sensor replay does not rerun physics.
python3 -m world_model_dataset.causal_observe \
  --episode output/world_model_dataset/v0_2/c2_none_next
```

Finalization follows a post-run `human_review.json`, not automatic behavioral thresholds.
No acceptance is inferred merely because a process exited or observations were produced.

```python
from world_model_dataset.causal_loader import open_episode

episode = open_episode("output/world_model_dataset/v0_2/c2_none_smoke02")
states = episode.states()
contacts = episode.contacts()
controls = episode.controls()  # legitimately absent for none
inputs = episode.resolved_inputs()
for frame, arrays in episode.observations():
    rgb = arrays["rgb"]
    depth = arrays["depth_m"]
    segmentation = arrays["segmentation"]
annotations = episode.annotations()
outcomes = episode.outcomes()
```

Default v0.2 loading enforces a complete candidate manifest and checks accessed artifact hashes.
That is record completeness, **not** a matrix, split or dataset-release admission. The candidate
completion record explicitly keeps both matrix and release admission false.

## Next step

Build the finite-force rigid-push smoke using the same resolved records, native timeline and
two-view replay. Then implement dynamic finite-impedance soft loading/compression with material
readback and Tet diagnostics; flexible contact impulses remain explicitly unavailable. After the
three smoke paths work, add the strict single-variable comparisons required for C2 acceptance.
