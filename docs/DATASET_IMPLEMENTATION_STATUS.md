# Dataset implementation status

Updated: 2026-09-17. Branch: `dataset/interaction-world-model-m4`.
Local-only work. No GitHub push or server submission. Fluid baseline unchanged.

## Causal redesign decision

The former R01--R05/V01--V05 catalogue mixed fixtures, terrains, material pairings,
interaction mechanisms, tasks and outcomes. It is no longer the ontology for dataset
production. M4 remains accepted as a **physics capability and regression corpus**, and the
M5A videos remain diagnostic visualizations of those caches. Neither status grants causal
training admission.

The replacement roadmap is
[Interaction World Model Dataset Plan v0.2](INTERACTION_WORLD_MODEL_DATASET_PLAN.md). It
factorizes an episode into system, environment, initial state and physical control, then
derives multi-label interactions and outcomes from the trajectory. Dataset Contract v0.1
and all historical evidence remain frozen. A separate v0.2 contract increment must reject
in-timeline state writes, object/collision removal, unlimited kinematic actors and render-only
hiding before scale production resumes.

## Milestones

| Milestone | Actual status | Evidence / remaining work |
| --- | --- | --- |
| M1 | Complete and frozen as v0.1 | Separate registries, JSON schema, strict JSON/path/hash/time checks, source snapshots, prepared examples, unified loader and single-variable checker. Release hashes are recorded in `contract_v0_1_release.json`. |
| M2 | Core framework complete | Four native action types, five fixture families, six diagnostic meshes, shared R01/V02 runner, state/contact caches and post-physics inspection. Legacy 14-environment/42-object fixed-topology data has a truthful read-only bridge; formal resimulation is deferred to M4 event migration. |
| M3 | Corrected physical scope complete; observations deferred | R01 remains accepted. Corrected V02: 9 material-readback-matched runs, 6 strict pairs and one bounded repetition. Historical 15 default-material V02 comparisons remain withdrawn. Recovery/reference correction and penetration limits are documented in `DATASET_V02_CORRECTED_REVIEW.md`. |
| M4 | Complete as physics regression corpus | 117 canonical matrix episodes, 81 strict pairs and 4 real-STL supplements are accepted for physics regression. Their causal-training status is intentionally unresolved. |
| M5A | Diagnostic showcase complete | Ten cache-only videos demonstrate replay and presentation. They are not v0.2 training observations; the old replay can hide a removed support. |
| C0 | In progress; contract and mechanical inventory implemented | The orthogonal ontology, separate `v0_2` JSON schema, no-control example and static causal auditor are present. All 121 historical caches now have immutable source hashes, a candidate crop boundary and a conservative reuse class; detailed interaction/object/observation review remains before C0 freezes. Frozen v0.1 files are unchanged. |
| C1 | In progress; translational effort and impedance passed | A dynamic pusher on a prismatic joint receives the same bounded command under free, resisted and overloaded conditions. Native trajectories diverge and the overloaded actuator stops at 100% late saturation with residual position error. Rotational control and field control remain. |
| C2 | Existing causal prototypes reused | This C4 subset reuses the completed no-control and finite-push caches; prior detailed prototype reviews retain their own scope. |
| C3 | Representative experiments complete in declared scope; full combinations remain open | P01–P12 simple-geometry pairs, selected real-asset batches, Blue Wall stop/fall and original warehouse push-obstruction/chain-propagation pairs. Original scene geometry remains in collision; this is no longer only the old warehouse background layout. See `NATIVE_SCENE_BATCH02_AND_C4.md`. |
| C4 | First internal rigid subset delivered; full C4 not closed | `c4_rigid_micro01`: 10 episodes, 1,460 RGB-D/instance-segmentation view frames, 5,770 physical states; source-family train/validation/test groups 4/2/4. Unified loader reads all ten, fixed physical caches remain byte-identical. See `NATIVE_SCENE_BATCH02_AND_C4.md` and output `delivery_review.json`; normals, motion vectors, flexible objects and public release deferred. |
| C5 | Not started | No model benchmarks or global OOD claims from the first small subset. |

## Current M4 result

`output/world_model_dataset/v0_1/m4_r03_physics_v02/` contains the first completed R03
matrix: 11/11 physics runs, with sphere, rounded cube and cylinder pairs. Each geometry has
a reference collision plus a right-body restitution and density counterfactual. The sphere
family also includes a deliberate lateral miss and a moving-body-versus-stationary-target
case. No rendering was generated.

The canonical R03 fixture is a symmetric finite low-friction collision lane. Both fixture
and body friction use the `min` combination rule, so floor support contributes no horizontal
friction while object-object friction remains active. Speculative CCD is disabled inside the
bounded R03 speed/time-step domain: its anticipatory positive-separation constraint had
suppressed restitution in an earlier diagnostic. At 240 Hz and the current 5 m/s upper speed,
the maximum single-step travel is well below the 0.1 m diagnostic body radius.

The sphere results agree with the one-dimensional analytic solutions: reference, effective
0.45-restitution, and 2:1 right-mass runs have kinetic-energy ratios 0.01000001, 0.20250065,
and 0.12000004. The stationary-target result gives post-collision speeds 0.675 and 0.825 m/s.
The lateral-offset case has zero interbody contacts, as intended. Rounded-cube and cylinder
energy ratios remain close to the corresponding sphere controls while retaining their native
small rotations and off-axis contact response. Contact absence is therefore an outcome
measurement, not a failure gate; cache integrity, finite state, action delivery, topology and
ground containment remain hard checks.

`physics_completed`, manifest `lifecycle=completed`, and `validation.passed=true` mean
different things. Completing simulation or packaging never implies dataset acceptance.
Consumers must use `validate --require-complete` / default `Episode(...)` to enforce
the acceptance report. Explicit `require_complete=False` is inspection-only.

Development now reviews physics directly from native caches before rendering. Automatic
checks remain for corrupt/missing data, invalid time axes, non-finite state, topology/Tet
failure, missing actions and required rigid contact truth. Surface/proxy overlap is retained
as an exact diagnostic measurement and interpreted together with native contact and motion;
it no longer acts as a universal millimetre threshold for every geometry.
The unified loader can inspect an audited physics-only rigid contact stream only when called
with `require_complete=False`; this does not promote the prepared cache to a released episode.

## Current M3 physics result

`output/world_model_dataset/v0_1/m3_physics_v01/` contains 35/35 completed physics runs:

- R01: 20 episodes across sphere, cylinder, rounded cube and capsule; 10/20/30 degree
  angle families and dynamic-friction pairs.
- V02: 15 episodes across 10%, 25% and 40% commanded compression; three Young's modulus
  profiles and two compression durations.
- 21 declared single-variable pairs have identical rest geometry and differ only at the
  declared semantic pointer. Fixture-angle pairs intentionally have different world pose
  because placement is derived from the changed ramp.
- Four R01 duplicate baselines are bit-exact. V02 duplicate aggregate compression differs
  by at most 0.0041 and final shape residual by at most 0.0012 mm, while transient nodes can
  differ by at least 6.40 mm; V02 is aggregate-repeatable, not trajectory-deterministic.
- All 15 V02 runs, totalling 18,000 physics substeps, have zero Tet inversions. The worst
  minimum Jacobian is 0.6376. Final shape residuals are 0.113--0.146 mm.
- Manual review is recorded in `manual_physics_review.json`; pair/replay scope is recorded
  in `physics_consistency_review.json`.

The old V02 displacement-controlled runs appeared not to produce a reliable monotonic
Young's-modulus response. That observation is no longer evidence about the prescribed
moduli: the backend was using one default deformable material for every run. Those caches
must not be advertised as a modulus-identification subset. V02 must be rerun through the
corrected material path before any material or speed conclusion is restored.

Correction discovered during M4: the former deformable material prim combined
`UsdPhysics.MaterialAPI`, `PhysxMaterialAPI`, `OmniPhysicsDeformableMaterialAPI`, and a
duplicate collision-child binding. PhysX simulated the body but its deformable-material
tensor view matched zero materials, so the solver silently used backend defaults. The M3
V02 modulus conclusion is therefore superseded. The corrected path follows the installed
`VolumeDeformableDemo.py`: only the Omni deformable API plus PhysX deformable extras are
placed on the material, and it is bound once at the body root. Native tensor readback is now
required to match declared modulus, Poisson ratio and dynamic friction. Corrected V01 probes
show a clear 30/100/300 kPa compression ordering (26.90%, 13.54%, 4.07%).

## Implemented entrypoints

CPU Python with numpy/scipy/trimesh/jsonschema:

```sh
python -m world_model_dataset.runner prepare configs/dataset/examples/r01_sphere.json output/world_model_dataset/v0_1/<new_id>
python -m world_model_dataset.runner validate output/world_model_dataset/v0_1/<new_id>/episode.prepared.json --check-source
python -m world_model_dataset.local configs/dataset/examples/r01_sphere.json output/world_model_dataset/v0_1/<another_new_id> --render
python -m world_model_dataset.audit output/world_model_dataset/v0_1/<native_cache>
python -m world_model_dataset.finalize output/world_model_dataset/v0_1/<audited_cache>
```

`local` uses a free-VRAM capacity guard (4096 MiB by default, configurable with
`DATASET_MIN_FREE_GPU_MIB`) and permits concurrent small GPU processes; it writes native
logs, checks both process status and the failure
report (SimulationApp can exit 0 after an exception), and only renders after physical
audit passes. Native physics uses Windows Isaac Python; CPU dependencies are not installed
into or assumed present in the Isaac environment. Output paths are never overwritten.

## Run ledger

All paths below are relative to `output/world_model_dataset/v0_1/`.

| Directory | Meaning |
| --- | --- |
| `m1_r01` | First prepared-only M1 sample. Source hashes predate later M2 edits; not a simulation. |
| `r01_native_probe01` | 4 s, 121 frames, 628 native contact points. Failed physical use: finite floor too short; object rolls off. Retained diagnostic. |
| `r01_native_probe02` | Floor extended; fixture material fixed independently from subject. 4 s, 121 states, 1,051 native contact points; current sampled-state physical checks pass. 242 two-camera RGB/depth/segmentation observations aligned and subject visible in all. Final validation deliberately false: full intersection audit, normals convention and motion-vector validation remain. |
| `v02_native_probe01` | Initialization failure: empty USD node velocities before first step. |
| `v02_native_probe02` | Initialization failure: CPU tensor view's nodal velocity accessor is not implemented in this runtime. |
| `v02_native_probe03` | Authored t=0 zeros explicitly distinguished; tensor accessor still unavailable after stepping. |
| `v02_native_probe04` | Corrected to native USD velocity readback after stepping. 5 s, 151 frames, no sampled Tet inversions; observed compression 18.13% for commanded 25%; up to 8.33 mm visible-surface/fixture overlap; zero soft-body native contact reports. Failed physical audit, not rendered. Early cache stores mean velocity, not full nodal velocity arrays. |
| `v02_native_probe05` | Full native nodal velocity arrays saved (2,197 nodes), explicit total mass, conforming remesh settings. 5 s, 151 frames; compression 17.98%; overlap 8.30 mm; zero sampled inversions; zero contact reports. Failed audit, not rendered. Multiple instrumentation corrections: not a single-variable physical comparison. |
| `v02_contact_shell_probe06` | Synchronized visual/collision/tet export plus native plate pose. Max visual overlap 8.40 mm; collision-node overlap 9.56 mm; plate target error only 1.02e-8 m. Confirms collider penetration, not merely skin mapping or bad plate trajectory. Failed audit, not rendered. |
| `m1_contract_current` | Prepared-only refreshed M1 sample with effective source/config snapshots. Validated at creation; later numerics registration requires a new preparation for execution against current code. |
| `m1_contract_current02` | Current M1 prepared sample including independent numerical profile and full effective source snapshot; read-only validation with `--check-source` passed. |
| `v02_contact128_probe07` | Single numerical change: deformable position iterations 32→128 at unchanged 240 Hz. 5 s, 151 states; visible overlap 1.116 mm, collision-node overlap 1.399 mm; compression 24.52% of commanded 25%; zero sampled inversions. Geometric checks pass; zero native soft contact reports still prevent acceptance/rendering. Physics loop took 78.7 s. |
| `m2_action_probe02.json` | Native probe passed all four action types and all five fixture families. |
| `legacy_bridge01.json` | Read-only inventory passed for 14 legacy environments, 42 objects and their fixed-topology caches. |
| `m3_physics_v01` | Historical 35-run M3 matrix: 20 R01 runs remain valid; 15 V02 caches remain mechanics/topology evidence under the backend default material but are superseded for declared-material comparisons. |
| `m4_r03_physics_v01` | Partial diagnostic using the inherited one-sided R01 floor; the high-restitution sphere left the finite negative-X edge. Superseded, retained as scene-design evidence. |
| `m4_r03_physics_v02` | Current R03 physics matrix: 11/11 complete, three geometries and eight strict single-variable pairs; manually reviewed from native state/contact caches. |
| `m4_v01_physics_v01` | Three-run diagnostic made before the material-registration bug was fixed; the authored modulus was not present in the native deformable-material view. Superseded and not dataset evidence. |
| `v01_rounded_cube_drop_smoke04` | Last pre-fix V01 mechanics smoke: verified drop/rebound/topology and the zero-friction fixture, but used backend default deformable material. |
| `v01_modulus_30k_probe01`, `v01_modulus_100k_probe01`, `v01_modulus_300k_probe01`, `v01_modulus_10mpa_probe04` | Corrected one-second V01 material probes. Native readback matches every declared material and deformation changes strongly with modulus. |
| `m4_v01_physics_v02` | Current V01 matrix: 11/11 complete, three geometries and eight strict single-variable pairs. All 11 native material readbacks match the declared values; no sampled or substep Tet inversion occurred. |
| `m4_r02_physics_v01` | Superseded R02 diagnostic matrix. It retained speculative CCD, which made the restitution counterfactual difficult to interpret in the bounded stair regime. |
| `m4_r02_physics_v02` | Current R02 matrix: 11/11 complete, rounded cube/sphere/capsule and eight strict material or stair-height pairs. Speculative CCD is explicitly disabled for R02. |
| `v05_sphere_soft_cube_impact_smoke01` | Initialization failure before first capture: missing Usd import. No usable physics result. |
| `v05_sphere_soft_cube_impact_smoke02` | First-step diagnostic readback failure: stopped-timeline CPU tensor view lacks nodal-velocity getter. No usable completed physics result. |
| `v05_sphere_soft_cube_impact_smoke03` | Completed mixed mechanics smoke using native USD nodal velocities, matching 100 kPa material readback, 181 shared captures, and 720 steps without Tet inversion. Retained initial shorter-floor diagnostic; a same-fixture hit/miss pair follows with extended support. |
| `v05_sphere_soft_cube_impact_smoke04` | Accepted extended-floor hit baseline. Ground length uses the declared speed domain, not the selected action speed, to keep speed and hit/miss controls on the same fixture. |
| `v05_sphere_soft_cube_miss_smoke01` | Accepted same-action/same-support lateral miss. Projectile retains 1.5 m/s; no sampled geometric target contact; target changes are settling-scale only. |
| `m3_v02_material_corrected_v01` | Accepted corrected physical scope: 9 runs, 10/25/40% compression crossed with 30/100/300 kPa, 128 position iterations, 6 strict pairs. Supplemental equilibrium recovery and bounded 25% / 100 kPa repetition evidence preserve original metrics. |
| `m4_v05_physics_v01` | Accepted 11-run V05 physical matrix: baseline, low/high speed, light/heavy projectile, soft/stiff target, eccentric/miss, capsule projectile and soft sphere. Eight strict leaf-level pairs; two geometry cases are standalone coverage. Zero inversions in 7,920 steps; worst minimum J 0.5892 and maximum sampled penetration 1.959 mm occur in the high-speed boundary case. |
| `r04_rounded_cube_obstacle_push_smoke01` | R04 layout diagnostic: a 6D continuing pusher traps the rounded cube against the obstacle. Retained as a stuck boundary, not the canonical deflection action. |
| `r04_rounded_cube_deflection_smoke02` | Superseded R04 action diagnostic: angled obstacle added but 2D pusher travel still ends inside the blocked object; abnormal late velocity makes it unsuitable as a normal deflection sample. |
| `r04_rounded_cube_inertial_deflection_smoke03` | Stable short-push diagnostic: no abnormal late velocity, but the reference-friction cube settles behind the obstacle. The original narrow side clearance is superseded before testing the pass branch. |
| `r04_rounded_cube_low_friction_deflection_smoke04` | Wider-lane, low-friction test remains stably wedged at the angled obstacle. This is geometry-dependent stuck behavior, not a pass branch or numerical failure. |
| `r04_sphere_inertial_deflection_smoke05` | Stable pass-branch smoke: pusher → angled obstacle → left rail, with 1.75D lateral displacement and no late velocity explosion. The first audit's X-only clear flag is superseded; formal R04 recognizes lateral clearance after passing the obstacle center. |
| `m4_r04_physics_v01` | Prepared 11-run R04 definition: sphere, rounded cube and capsule; eight strict pairs over friction, push duration, obstacle offset and subject offset. Includes mirrored routing, wide miss and geometry-dependent stuck/pass outcomes. |

The first completed V05 mechanics smoke identifies geometric contact at 0.633--0.692 s.
The projectile changes from 1.5 m/s to approximately 0.054 m/s (a small reverse velocity),
while the free soft target is translated. Every-step local nonrigid displacement peaks at
21.11 mm; minimum Tet Jacobian is 0.7010 with no inversion. A cache-only preview at the
shared 0.650 s / step 156 confirms both meshes occupy the same impact frame. Global X extent
compression is almost zero in this run despite substantial local deformation; it is an
extent diagnostic, not a substitute for local compression/strain. The collision-node/sphere
penetration estimate (0.242 mm) is a sampled lower bound, not exact solver penetration.
Native contact records here cover projectile/floor only. Rigid/soft force and impulse remain
unavailable, and no impact impulse is estimated from the velocity change.

The reviewed V01 matrix shows a useful material signal without rendering. For rounded cube,
sphere and capsule respectively, 30/100/300 kPa produce maximum compression of
26.90/13.54/4.07%, 36.80/19.70/12.25%, and 22.12/4.42/3.48%. Rounded-cube drop height
0.5D/1.5D/3.0D produces 7.50/13.54/17.34% compression. Maximum lateral COM drift is
0.072 mm across the matrix; the minimum every-step Tet Jacobian is 0.441768 and remains
positive. Soft-contact impulse remains explicitly unavailable and is not inferred.

The reviewed R02 matrix uses five 0.3D-high, 1.5D-deep treads and a one-shot 1.4 m/s
initial velocity after 0.5 s of rest. The sphere contacts all five steps and the floor, then
continues rolling. The rounded-cube reference reaches `step_3`, while its low-friction and
high-restitution variants settle on `step_2` through different orientation branches. The
capsule reference and low-friction runs settle on `step_2`; the high-restitution run reaches
`step_3`. The 0.2D and 0.5D height variants remain finite and stable. Sphere restitution is
a deliberately difficult comparison in this geometry because the sphere rolls across tread
edges with little normal closing velocity. No required outcome category is used as a gate.

The first rigid render launcher was stopped during its CPU audit/restart attempt;
the native renderer had already started and finished independently. All output hashes,
frame counts and segmentation visibility were subsequently checked. No physics rerun
was hidden in the rendering stage.

## Contact capability gap

This installed PhysX version's `PhysxContactReportAPI` documentation describes rigid
bodies/articulations. Probe code puts the API on the deformable root, collision mesh,
and rigid plates, threshold zero, and subscribes once for every explicit physics step.
The rigid control reports points correctly, whereas both complete compression probes
return zero headers, despite clear deformation and plate motion. This establishes a
gap in **this integration**, not a claim that no PhysX version or custom native extension
can ever expose deformable forces.

References:

- Installed: `omni.physx-110.1.13/.../docs/dev_guide/contact_reports.rst` and `ContactReportDemo.py`.
- [Official ContactReportAPI definition](https://docs.omniverse.nvidia.com/kit/docs/usdrt.scenegraph/7.6.1/api/classusdrt_1_1_physx_schema_physx_contact_report_a_p_i.html).
- [PhysX developer's historical deformable reporting limitation](https://forums.developer.nvidia.com/t/how-to-grasp-deformable-object/312694/4); historical context, not sole evidence for this installed version.

Do not populate contact forces from penetration depth or position differences. Do not
call an empty unsupported stream “zero force”. Callback contact impulse is preserved
as reported; full friction-anchor accounting and impulse/normal sign validation remain
before publishing force–displacement or momentum-balance labels.

## Historical carry-over limits

- The bounded 0.1 s rapid-withdrawal probe is complete. It activates free oscillation but
  does not produce a robust monotonic modulus label at 30 Hz. Material identification moves
  to a later high-rate free-response or force-observable event; no further V02 sweep.
- Native nodal velocity under nonidentity deformable-root rotation remains unverified;
  current V02 scope uses identity orientation. COM/energy use documented lumped rest-Tet
  volume weights.
- Bulk observations are deferred. The two-camera RTX path, world-normal convention,
  derived fixed-topology material motion and cache-only Cycles path have individual smoke
  evidence; generate them only for selected accepted caches when observations are needed.
- Legacy runs remain a read-only bridge until their event is intentionally migrated and
  resimulated under the v0.1 truth contract.

## Checks completed

- 25 dataset CPU tests pass (contracts, invalid inputs, snapshots, frozen release hashes, strict pairing,
  geometry, fixture scale, nonrigid shape removal, censored recovery and motion semantics).
- 9 existing repository-layout tests pass.
- 6 existing tetrahedral-quality tests pass.
- 40 total selected tests pass.
- Four native actions and five fixtures pass the native capability probe.
- R01/V02 physics matrix, R01 two-camera output and cache-only Cycles smoke checks are
  described above.

M1--M4 and the M5A showcase are closed only in their declared historical scopes. They remain
valid regression evidence, but do not define the v0.2 ontology or grant causal-training
admission. Broad scale production remains paused; the explicitly selected C4 rigid micro-subset
now advances with existing finite actuators and source-family splits, without waiting for all
historical C0 migration or full C3 coverage. Material identification remains assigned to a future
force-observable or high-rate free-response study, not historical V02.
