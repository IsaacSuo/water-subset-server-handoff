# C1 finite actuator capability review

Status: first capability complete, C1 remains in progress (2026-09-16).

## Probe

`c1_bounded_effort_probe01` tests one physical pusher constrained by a prismatic joint. The pusher
is a dynamic rigid body with mass and inertia. A velocity-feedback effort controller may apply at
most 8 N through `PhysxForceAPI`; it never writes pose or velocity. Three lanes receive the same
0.8 m/s command from 0.2 to 1.2 seconds:

- `free`: no load;
- `resisted`: a movable 0.5 kg load;
- `overload`: the same load backed by a fixed wall.

The accepted local run is
`output/world_model_dataset/v0_2/c1_effort_probe03`. It ran at 240 Hz for 1.8 seconds and records
the command, native actuator state after every physics step, requested/applied external-force
input, saturation, signed work and contact impulse. No rendering was generated.

## Result

| Scenario | Displacement during command | Velocity at command end | Late saturation | Pusher contact impulse |
| --- | ---: | ---: | ---: | ---: |
| Free | 0.7524 m | 0.7999 m/s | 0% | 0 N·s |
| Resisted | 0.7142 m | 0.7497 m/s | 0% | 1.1999 N·s |
| Overload | 0.4000 m | 0.0010 m/s | 100% | 4.2999 N·s |

The overloaded actuator reaches the load/wall stack, remains force-limited and stops rather than
following the free trajectory. This is the required reaction-sensitive behavior: identical command
and controller parameters produce different actual trajectories under different loads.

The completion report SHA-256 is
`f2607e783edaf69b88d5de104c3a340f662d3253fc482b1b3b7c2fcdbf253a3d`. Command, state and effort
trace hashes are respectively
`18502ae8e8fccf6d383dca117fd12bf8c141f52192d7110eaaafcd47e76bdd78`,
`c2759ff2938a33490700954d1907cc3fb60a530e12cb660f7e9cd1986684dde8` and
`62aaf4f81199d173513420180068707a71c116233e619ab87fc99646b34bf887`.

## Contract feedback

The probe exposed two fields that the first v0.2 draft needed to state explicitly:

- body pose and velocity belong to `initial_state`, not `system`;
- actuator constraints need declared joint bodies, axis and finite limits.

The draft schema and both examples now use that separation. The controlled example is
`configs/dataset/v0_2/examples/effort_pusher_overload.json`.

## Limits and next work

This proves the finite translational effort layer only. The effort trace is the bounded external
force submitted to PhysX and the actuator state is native; joint/contact reactions are inferred
only where native contact impulses are available and are not relabelled as a joint force sensor.
C1 still needs:

1. a finite linear impedance drive with target/actual/error/effort records;
2. a rotational effort or impedance probe;
3. a continuous field-control probe;
4. packaging each lane as a complete v0.2 episode instead of a combined capability run.

The first failed directory, `c1_effort_probe01`, contains only an import error. Probe02 has valid
physics but used a whole-command saturation threshold that misclassified the overload because the
first half of the command was free travel. Probe03 changes only that review statistic: overload is
judged from the final command quarter and near-zero actual velocity.
