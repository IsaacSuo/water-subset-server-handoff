# Dataset implementation status

Updated: 2026-09-15. Branch: `dataset/interaction-world-model-m4`.
Local-only work. No GitHub push or server submission. Fluid baseline unchanged.

## Milestones

| Milestone | Actual status | Evidence / remaining work |
| --- | --- | --- |
| M1 | Complete and frozen as v0.1 | Separate registries, JSON schema, strict JSON/path/hash/time checks, source snapshots, prepared examples, unified loader and single-variable checker. Release hashes are recorded in `contract_v0_1_release.json`. |
| M2 | Core framework complete | Four native action types, five fixture families, six diagnostic meshes, shared R01/V02 runner, state/contact caches and post-physics inspection. Legacy 14-environment/42-object fixed-topology data has a truthful read-only bridge; formal resimulation is deferred to M4 event migration. |
| M3 | R01 complete; V02 cache mechanics retained but material runs superseded | A later M4 native tensor readback found that the old mixed rigid/deformable material prim was not registered by the PhysX deformable backend. The 15 V02 runs remain evidence for actions, topology and cache export under the backend default material, but not for their declared 30/100/300 kPa inputs. |
| M4 | In progress; R03, V01 and R02 initial matrices complete | Each event has 11 physics episodes over three geometries and eight strict pairs. V01 includes native deformable-material readback. All 33 episodes were reviewed from physics caches without bulk rendering. |

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

`local` refuses busy GPU, writes native logs, checks both process status and the failure
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

## Remaining scoped work after M3

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

M1--M3 are closed in the declared scope. M4 R03 is now implemented and its initial physics
matrix is complete; V01 soft-body drop/rebound is next. Material identification remains
assigned to a future force-observable or high-rate free-response event, not V02.
