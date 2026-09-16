# Dataset v0.2 causal migration audit

Status: preliminary event-family audit, 2026-09-16.

This audit classifies the 121 caches accepted by the historical M4 physics review. It does
not change their physical acceptance, files or hashes. It asks a different question: whether
the full recorded interval can serve as causal world-model training data under the v0.2
roadmap.

## Classification

| Historical family | Count | Existing mechanism | Full-interval causal status | Permitted v0.2 reuse |
| --- | ---: | --- | --- | --- |
| R01 | 20 | gate collision disabled at release time | ineligible | Candidate post-release `none` segment if the first derived state, object set and visible geometry are made self-consistent |
| R02 | 11 + 1 real | velocity written after a recorded rest interval | ineligible | Candidate `none` episode beginning at the first complete post-write state |
| R03 | 11 + 1 real | two velocities written after a recorded rest interval | ineligible | Candidate `none` episode beginning at the first complete post-write state |
| R04 | 11 + 1 real | prescribed kinematic pusher trajectory | ineligible as controlled interaction | Physics regression; a post-actuation passive segment may be considered separately but cannot retain the push claim |
| R05 | 11 + 1 real | support removed/disabled during the interval | ineligible | Candidate post-removal `none` collapse episode only if the support is absent from the derived system from its first state |
| V01 | 11 | gravity-driven free fall; no synthetic release command | direct candidate | Full interval may enter detailed state-continuity and observation audit |
| V02 | 9 | prescribed kinematic plate compression/release | ineligible as controlled interaction | Material/deformation regression; a separately defined passive recovery segment may be considered |
| V03 | 11 | load release followed by support/removal action | ineligible | Load/recovery regression; a separately defined passive segment may be considered |
| V04 | 11 | prescribed kinematic pusher trajectory | ineligible as controlled interaction | Confinement/deformation regression; post-actuation passive motion only if useful |
| V05 | 11 | projectile velocity written after a recorded rest interval | ineligible | Candidate `none` episode beginning at the first complete post-write state |

The counts sum to 121. “Candidate” does not mean admitted. Every derived episode needs a new
identity, parent-cache hash, exact crop boundary, regenerated time origin, initial-state
declaration and observation review. A crop may preserve a passive physical trajectory; it
cannot rename an old kinematic or removal action as a finite physical actuator.

## M5A observations

All ten M5A videos remain diagnostic only. Their source intervals inherit the action status
above. In addition, the R05 renderer hides the support after the historical removal command,
so the rendered object set is not a faithful continuous observation of a v0.2 physical
system. No M5A video is currently admitted to v0.2 training or evaluation.

## Required cache-level audit

For each candidate family:

1. identify the last preparation or direct-state mutation step;
2. require a complete state sample immediately after it and before the first relevant contact;
3. verify that all later state changes come from the solver;
4. define the derived system's object and collision set at its new time zero;
5. regenerate action metadata as `none`; do not retain the historical command;
6. preserve parent paths and SHA-256 values without editing the parent cache;
7. reject the crop if the remaining interval is too short, begins after the interaction, or
   cannot produce visually and physically consistent observations.

## Consequence

The old M4 corpus remains valuable even if few full episodes migrate. It covers solver
behavior, materials, contact streams, Tet validity, geometry branches and output plumbing.
The next production corpus must be generated through finite embodied actuators or valid
initial conditions rather than by relabeling these historical actions.
