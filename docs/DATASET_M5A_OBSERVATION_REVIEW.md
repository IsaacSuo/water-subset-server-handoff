# M5A observation and rendering slice

Reviewed 2026-09-16 from accepted M3/M4 native caches. M5A establishes one neutral
observation domain and one representative cache-only video for each of R01–R05 and V01–V05.
It does not rerun physics and does not change any accepted physical result.

## Frozen presentation standard

- Environment: `canonical_studio_m5a`, dark neutral world, matte neutral floor, gray fixtures
  and orange actuators. This keeps event geometry readable without introducing a scene-specific
  visual cue.
- Camera roles: static `hero`, `analysis_side`, `profile` and `overview` views. All use a 52 mm
  lens, 36 mm sensor and event-level framing expressed in characteristic size `D`. A camera never
  follows an individual object or changes inside a counterfactual family. V04 mirrors the common
  hero azimuth around the action axis so the incoming object is not hidden behind the aperture.
- Lighting: one warm key, one cool fill, one rim and a weak sun with fixed colors and relative
  placement. Exposure, depth of field and motion blur are fixed rather than fitted per episode.
- Delivery video: Blender EEVEE Next, 1280×720, 30 fps, H.264, yuv420p, CRF 18. Cache samples are
  selected by physical timestamps, so 60 Hz caches are downsampled without changing playback time.
- Appearance: diagnostic objects retain their manifest appearance. The four named real assets use
  an explicitly recorded presentation palette; this is an observation-only appearance variant and
  must not be confused with a change to physical material.

The machine-readable source of truth is
`configs/dataset/m5a_observation_slices.json`. It fixes all ten episode selections, meaningful time
windows, event framing, lighting and video encoding. The renderer supports both single-object and
multi-object cache layouts; this is required for the banana collision and elephant-topped stack.

## First ten-event slice

| Event | Slice | Representative content |
| --- | --- | --- |
| R01 | `R01_sphere_ramp` | sphere released on a 20° ramp |
| R02 | `R02_carrot_stairs` | real carrot tumbling down five steps |
| R03 | `R03_banana_collision` | two real bananas in a head-on collision |
| R04 | `R04_chair_obstacle` | real chair pushed into the obstacle lane |
| R05 | `R05_elephant_stack` | elephant-shaped real asset atop a support-removal stack |
| V01 | `V01_soft_drop` | volumetric rounded cube drop and recovery |
| V02 | `V02_soft_compression` | corrected-material 25% plate compression and release |
| V03 | `V03_weight_loading` | rigid load placement, hold and removal over a soft target |
| V04 | `V04_soft_aperture` | soft cube driven through the converging aperture |
| V05 | `V05_rigid_soft_impact` | rigid sphere impact on a volumetric soft cube |

The output root is `output/world_model_dataset/v0_1/m5a_showcase_v01`. Each slice contains its PNG
frames, `render_report.json`, an H.264 video and `video_report.json`; the root `summary.json` indexes
all ten. The four priority deliverables are R02 carrot, R03 banana, R04 chair and R05 elephant.

Manual frame review checked the beginning, interaction and post-interaction portions of every
slice. It caught two presentation-only defects before closure: R05 originally continued drawing a
support after the remove-support action, and the initial V04 hero azimuth hid the incoming soft
object behind the aperture. The final renderer hides removed supports and uses the corrected V04
azimuth. These fixes affect only observation replay.

This closes the first M5A slice, not full Dataset v0.1 observations. The other fixed camera roles,
RGB-D/segmentation/normal/motion outputs, additional representative trajectories and environment
variants remain later M5 work.
