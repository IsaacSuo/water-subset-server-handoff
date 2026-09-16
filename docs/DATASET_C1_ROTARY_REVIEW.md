# C1 rotational actuator review

Reviewed locally on 2026-09-16. **The current C1 capability scope is complete**: bounded
translation and rotation, effort and impedance feedback, load reaction, independent
physics packaging and translational reproduction. C2 prototypes remain unimplemented.
No rendering, GitHub push or remote job submission was performed.

## Native implementation

The shared native probe now supports a 1 kg box rotor (1 × 0.1 × 0.1 m) on a Z-axis
revolute joint. It is driven by external `PhysxForceAPI` torque, not a joint pose target
or a kinematic trajectory. Joint limits are ±150 degrees; manifests and all runtime
angular traces use radians. The stationary non-colliding anchor is never moved.

This is a horizontal capability fixture: gravity is disabled for the rotor and movable
load. That exception is explicit in the initial USD scene and runtime report; these
results do not establish behavior of a gravity-loaded production mechanism.

Two feedback laws share a **0.2 N·m hard input limit**:

- Angular velocity effort: 0.8 rad/s target, 0.6 N·m·s/rad gain.
- Angular impedance: 1.2 rad target, 1 N·m/rad stiffness, 0.3 N·m·s/rad damping.

The command acts from 0.2 to 1.7 s; after that torque is zero, not a hidden brake or
position lock. Each scenario runs in its own native process for 2.2 s at 240 Hz.
Full-body state includes t0 and all 528 steps (529 records).

## Results

Run directory: `output/world_model_dataset/v0_2/c1_rotary01/`.

| Law | Load | Angle at command end (rad) | Angular speed (rad/s) | Input work (J) | Late saturation |
| --- | --- | ---: | ---: | ---: | ---: |
| Velocity effort | Free | 1.036690 | 0.794405 | 0.030025 | 0% |
| Velocity effort | Movable load | 0.882404 | 0.769943 | 0.080456 | 0% |
| Velocity effort | Wall-backed load | 0.364598 | −0.000077 | 0.026990 | 100% |
| Angular impedance | Free | 1.389723 | 0.091636 | 0.008661 | 100% |
| Angular impedance | Movable load | 1.174657 | 0.637531 | 0.126898 | 0% |
| Angular impedance | Wall-backed load | 0.364598 | 0.000025 | 0.072920 | 100% |

The movable load reduces angular progress for the velocity law and requires more work
under both laws. Wall-backed motion stops at the load, with native rotor/load contact
impulses of 0.496 and 0.649 N·s. Angular impedance retains **0.835402 rad** target error.
The stopping angle is far inside the joint limit, so a joint stop cannot explain it.

**Saturation is not itself an obstruction label.** The free impedance rotor overshoots
the target by 0.189723 rad, and late saturation is negative braking torque. This short,
underdamped profile proves finite feedback authority; it does not pass a settled-position
test and is not promoted to a production positioning recipe. On release, free motion
continues because external torque goes to zero. Those outcomes are retained unchanged.

Input work is the signed discrete torque × native angle increment diagnostic. External
input is not total torque, a joint-reaction sensor or calibrated real-world force truth.

## Direct numerical cache review

- Complete body sets remain constant; all quaternions are normalized to floating precision.
- Torque never exceeds 0.2 N·m and is zero outside the command interval.
- Floor, wall and anchor displacement is zero; worst rotor pivot drift is 0.00183 mm.
- Worst native contact separation is −0.00620 mm; all states are finite.
- Actual angles stay inside the ±2.617994 rad joint limits.
- Free initial acceleration implies 0.084184 kg·m² inertia, close to the uniform-box
  diagnostic value 0.084167 kg·m² (including the first-step damping difference). This is
  a sanity check, not a solver tensor readback or a convergence claim.

Raw completion summary SHA-256:
`a15346261adec3a8f0bf2f532d3093b66b46a8efcc159bcae95faaf2f827ef66`.
The raw summary retains its pre-review status; `human_review.json` records this review
separately rather than rewriting the original physics completion report.

The original linear overload probe was rerun through the extended native implementation
as `output/world_model_dataset/v0_2/c1_linear_regression01`. All 432 state samples and all
pre-existing numerical summary fields match the original reference exactly. Full tests:
125 passed; frozen v0.1 artifacts remain unchanged.

## Reproduction and remaining scope

```bash
python3 -m world_model_dataset.c1_rotary \
  --output output/world_model_dataset/v0_2/c1_rotary02
```

The output directory must be new. Every episode includes resolved config, initial/final
USD stages, complete native body state, actuator/command/effort streams, native point
contacts, source snapshots and artifact hashes. Manifests remain physics-only drafts;
observations, derived labels and dataset admission are not implied by C1 completion.

Next is C2: legal-initial-state no-action collision, finite-force rigid push, and finite-
impedance soft loading/compression, with aligned state/control and two observation streams.
The six rotational load comparisons do not replace any of those prototypes. C0 is still
a draft pending its migration/runtime-record review; C1 completion does not freeze the contract.
