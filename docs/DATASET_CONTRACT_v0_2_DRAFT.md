# Dataset Contract v0.2 causal increment (draft)

Status: C0 review draft, 2026-09-16. This document and its schema do not modify or supersede
the frozen v0.1 files. They define the admission boundary for newly generated causal episodes.

## Purpose

The v0.1 contract can describe a simulation cache, but its event/action vocabulary also permits
mid-timeline velocity writes, collision removal and prescribed kinematic trajectories. Those
operations are useful for solver regression and are not automatically suitable for causal world-
model training.

The v0.2 increment separates:

```text
System + Environment + InitialState + ControlProgram
       -> PhysicalTrajectory -> InteractionAnnotations + Outcomes
```

The machine-readable draft is `configs/dataset/v0_2/schema.json`. A minimal no-control collision
is `configs/dataset/v0_2/examples/rigid_collision_none.json`. Static semantic checks live in
`world_model_dataset/causal_contract.py`.

## Contract sections

- `system` declares every subject, actuator and environment body that participates in physics.
  Initial pose and velocity belong to each body here and are already true at formal time zero.
- `environment` identifies boundary bodies and fixed external conditions such as gravity. It does
  not imply an action or an outcome.
- `initial_state` records the complete participant set and distinguishes a fresh simulation from
  an immutable crop derived from a historical cache.
- `control_program` contains one of `none`, `effort_control`, `impedance_control` or
  `field_control`. Commands, controllers and actual actuator traces are separate records.
- `trajectory` points independently to state, contact, observation, interaction-label and outcome
  streams. An unavailable backend truth is explicit and is never represented as an empty stream.
- `implementation` declares who may update state and lists post-origin operations. This makes
  causal shortcuts statically rejectable before simulation.
- `lineage` retains the exact parent manifest hash and crop interval for every derived episode.
- `counterfactual` keeps the strict single-variable pairing structure without using historical
  event IDs as the ontology.

## Static causal admission

The draft auditor rejects an episode when any of the following is declared after formal time zero:

- direct pose, velocity or deformable-state writes;
- body spawn, deletion, activation or deactivation;
- collision enable/disable;
- render-only show/hide changes;
- an unbounded kinematic target;
- a changing physics participant set;
- a controlled actuator without finite effort authority, physical reaction visibility, command
  trace, state trace or applied-effort trace.

It also requires the initial-state participant set to equal the physical system, validates time
grids and command intervals, and enforces exact parent lineage for derived crops.

Run the draft audit with:

```bash
python3 -m world_model_dataset.causal_contract \
  configs/dataset/v0_2/examples/rigid_collision_none.json
```

Use `--require-complete` only for a produced candidate. It additionally requires completed state,
observation, interaction and outcome records; controlled episodes must also contain command,
actuator-state and applied-effort records.

## Meaning of acceptance

Draft acceptance means the manifest is structurally and causally admissible. It does not prove
that the declared implementation matches runtime behavior, that physics is numerically stable, or
that an intended outcome occurred. Runtime C1/C2 work must compare declarations with native state
and control traces. Manual review remains responsible for judging the physical behavior after each
prototype run.

## Compatibility and freeze boundary

- `configs/dataset/schema.json` and all files hashed by `contract_v0_1_release.json` remain frozen.
- A v0.1 cache is never edited in place or silently promoted.
- A migrated cache receives a new v0.2 episode ID and parent hash.
- Historical action names are not copied into `control_program`; a valid passive crop uses `none`.
- Historical kinematic/removal actions remain regression evidence unless a new physical actuator
  reproduces the interaction.

## C0 status after this draft

The ontology, schema draft, no-control example and negative static checks now exist. A mechanical
inventory also resolves all 121 historical episodes to immutable source hashes and candidate crop
boundaries. C0 is not yet frozen. Remaining work is the detailed interaction/object/observation
review of those candidates, a controlled manifest example exercised against the first C1 probe,
and a final review of runtime trace fields before assigning a stable `0.2.0` version.
