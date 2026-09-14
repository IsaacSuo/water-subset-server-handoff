# Dataset Contract v0.1

M1 contract source: `configs/dataset/schema.json`; CPU validator:
`python -m world_model_dataset.runner validate <episode.prepared.json>`.
Only `prepare` creates a directory; validation never edits inputs or launches GPU work.

## Identity and provenance

`spec` declares event, environment, cameras, parameters, seed and an ordered list of
objects. Each instance binds geometry, physical profile and appearance **by separate
IDs**. This replaces error-prone parallel arrays of object/material IDs from the roadmap.
`inputs` snapshots all resolved entries. `inputs_sha256` hashes canonical UTF-8 JSON.
`source_commit` plus per-file SHA-256 records the effective source, including uncommitted
work; a commit alone is not sufficient provenance. Published manifests are not overwritten.
New preparations archive the effective source/config bytes under `source/` and verify
each hash immediately, so uncommitted development snapshots are recoverable as well.

Registered materials are experimental parameter profiles, not calibrated real materials.
Numerical solver settings are a separate `numerics_profile_id` reference (default
`reference`), resolved from `numerics.json`; a solver convergence study is not a
physical-material counterfactual. Effective numerical values are snapshotted in inputs.
Geometry D is **maximum rest-pose axis-aligned extent**, in metres; density is held fixed,
so mass follows volume. Constant-D is not constant-mass normalization. Rotating a body
does not change its D. Collider choice is part of geometry, not its visual appearance.

## Time, units and actions

SI; right-handed world Z up; quaternion `xyzw`; angular state in rad/s (USD native
degrees are converted explicitly). Integer physics steps define `time_s=step/hz`.
The capture rate divides the physics rate. State at step 0 is pre-action. One-shot
commands at time t apply **before** the step [t,t+dt]; returned state is at t+dt.
Contact impulses belong to that step, not the nearest RGB frame. They have units N s,
not N; impulse/dt may be published separately as a **derived step-average**, never as
an instantaneous measured force. Action trajectories are functions of seconds, not
render-frame numbers. Rendering must not advance the physical state.
PhysX may emit a lost-contact header with an empty actor/collider path after removal;
this is preserved, not replaced with a fabricated object identity.

## Files and lifecycle

`episode.prepared.json` + `action.json` are inspectable M1 inputs. They are not a
completed episode and fail `validate --require-complete` by design.

Completed runtime writes `episode.json`, `state/index.json`, time-stamped state JSON
and fixed-topology NPZ data, `contacts.jsonl`, `observations/index.json`, `metrics.json`,
and `validation.json`. Artifacts use confined, portable, relative paths and SHA-256.
NaN/Infinity, duplicate JSON keys, absolute paths, parent traversal, escaping symlinks,
wrong hashes and mismatched times are rejected. No pickle loading is allowed.

`capabilities` distinguishes native, derived, unavailable, not-probed, not-applicable.
An empty contacts file does **not** prove the absence of contact when the representation
has no supported reporting interface. Missing required truth or observations prevents
M3 acceptance. Failure episodes remain available with explicit failure reasons.

Physics development may deliberately stop after `physics_validation.json`. Such a directory
is a reviewed physics cache, not a completed dataset episode, and the loader must open it
with explicit inspection mode (`require_complete=False`). Observations and `episode.json`
can later be generated from the accepted cache without advancing or rerunning physics.

## Observations

Two fixed calibrated cameras. Each observation records time, camera pose and intrinsic
matrix, RGB, optical-axis depth in metres, and integer semantic labels with ID mapping.
Normals and motion vectors must declare their coordinate system and units. Until an
annotator is verified, its channel is unavailable rather than zero-filled. Background
depth uses a validity mask, not non-finite JSON. Extra render updates use a stopped
timeline, so sensor warm-up never inserts hidden physical steps.

## Counterfactuals and splits

A family shares initial geometry, seed, timing, fixture and appearance except its one
declared semantic JSON pointer. The validator compares **resolved values**, not merely
different profile IDs. Pairing cannot silently change both static and dynamic friction.
Prototype samples belong to development only. All physical pairs and visual rerenders
remain in the same split family. 360 is a planning target, not yet a frozen pair count.

## M1 local check

```sh
python -m unittest discover -s tests -p 'test_dataset_*.py'
python -m world_model_dataset.runner prepare configs/dataset/examples/r01_sphere.json output/world_model_dataset/v0_1/m1_r01
python -m world_model_dataset.runner validate output/world_model_dataset/v0_1/m1_r01/episode.prepared.json --check-source
```

M2/M3 extend the runtime under this contract. The old mixed-body entrypoint and fluid
branch remain intact; this module does not import their scripts with launch-time side effects.
