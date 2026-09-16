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

## Cache-level mechanical inventory

The first read-only cache pass is recorded in
`configs/dataset/v0_2/historical_cache_inventory.json`. It resolves every episode through the
accepted matrix files, stores the SHA-256 values of its historical manifest, action stream and
state index, and selects the first complete captured state strictly after the last historical
state mutation. The inventory contains exactly 121 unique rows and no missing crop boundary.

| Mechanical class | Count | Meaning |
| --- | ---: | --- |
| `direct_candidate` | 11 | No historical action command; full V01 interval can proceed to detailed review. |
| `post_write_crop_candidate` | 35 | R02/R03/V05 plus real supplements; candidate time zero is the first captured state after velocity injection. |
| `post_removal_crop_review` | 32 | R01/R05 plus real supplement; requires explicit object, collision and observation-set review. |
| `post_actuation_passive_review` | 32 | R04/V02/V04 plus real supplement; only the passive tail may be useful and the old push/compression claim is discarded. |
| `post_deactivation_recovery_review` | 11 | V03; the passive soft-body tail requires special review because the load actor was deactivated. |

Every row deliberately has `automatic_admission=false` and `manual_review_required=true`. The
mechanical pass proves that a state sample and at least 1.48 seconds of historical trajectory exist
after the proposed boundary; it does not yet prove that the crop begins before the relevant
interaction, that the remaining object set is visually self-consistent, or that the tail is useful.

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

Items 1, 2, 5 and 6 now have machine-readable candidates for all 121 rows. Items 3, 4 and 7 remain
the manual/data-level admission pass. New v0.2 manifests have not been generated from these rows.

## Consequence

The old M4 corpus remains valuable even if few full episodes migrate. It covers solver
behavior, materials, contact streams, Tet validity, geometry branches and output plumbing.
The next production corpus must be generated through finite embodied actuators or valid
initial conditions rather than by relabeling these historical actions.
