# Volume Maps pressure projection A/B

The original-input baseline is
`output/coupled_scenes/active_transfer_akinci_1p0_volume_maps_fullsolver_4p1667_gpu2_20260929_v2/simulation`.
It completed 4.1667 s but still produces extensive droplets after hitting the receiver.

## Verified implementation issues

- The divergence deficiency mask initially zeroed pressure and excluded residuals,
  but the old Jacobi update could regenerate a deficient particle's pressure.
  The update now keeps its own pressure exactly zero for the entire projection.
  Neighbor pressure forces still act on it.
- Bender2019 maps do not register boundary point sets in the inspected
  SPlisHSPlasH implementation. The 20-neighbor threshold therefore counts only
  fluid neighbors; no equivalent boundary-neighbor heuristic is introduced.
- Volume Map gradients already enter the central-gradient square in alpha.
  Boundary gradients do not add separate neighbor-gradient squares.
- The current GPU solve has no previous-substep pressure warm start.
- The old adaptive acceptance checked finite state and CFL but not final
  projection residual. Jacobi cap counts alone did not distinguish convergence
  on the final iteration from true exhaustion.

## Runs

Both runs use the baseline's actual input, 4-mm spacing, 1200-Hz base step,
Akinci coefficient 1.0, iteration limits/minima, and minimum-step divisor.
The baseline runner used divisor **64**. Both runs start at t=0 and end at
t=4.1667; no cached warm start is used. Collision statistics cover 3.5–4.1667 s.

- A: fix pressure lock-zero; retain moving-boundary convergence scope and old
  CFL retry/recovery behavior. All-boundary final residual is observed.
- B: A plus all-boundary local convergence, at most four pressure-driven
  halvings, and 1.5 step recovery. The existing tolerances remain unchanged.
  Nonconverged finite CFL-safe trials at the halving budget or floor are
  accepted with an explicit flag. Nonfinite/CFL-unsafe trials never use this
  fallback. Full density and divergence solves are rerun after rollback.

The rollback restores stable-ID fluid position/velocity, Newton pose/velocity/
forces/time, the pending divergence reaction forces and torques, wrench records,
and cumulative prescribed work/impulse. Verification checks the restored fluid,
Newton state, and pending reactions exactly. The existing balanced frame-tail
rule is retained during pressure retries.

## Outputs and interpretation

`pressure_substeps.jsonl` records every attempted projection, its final
compression residual/tolerance, acceptance, iteration cap and pressure retry.
The report includes the number of explicitly accepted nonconverged substeps.
`receiver_rebound_events.json` records the first inward-to-outward normal-velocity
crossing of each stable ID inside the receiver map support. Incident speed is
the largest previously sampled inward relative-normal speed in that contact
episode. Per-frame wall density reports mean/P95 rho/rho0 in the first 4-mm layer
at each solid; the empty receiver before pouring is reported with zero count.
Density-source-positive deficient particle counts are recorded separately.

The comparison exports mechanical energy and reported prescribed motor work,
deficient particle counts on the same saved particle-cache basis, and matched
original-appearance videos. No physical pass/fail thresholds are added.

Interpretation limits:

- A rebound ratio above one alone does not prove energy injection: pressure
  exchanges energy between particles, redirects flow, and incident normal speeds
  can be small. Substep samples also limit the precision of crossing speeds.
- Fluid kinetic plus gravitational energy excludes Akinci surface potential.
  The existing Newton prescribed-work ledger consumes divergence reactions in
  the following substep, so it is not an exact instantaneous energy budget.
- New receiver pressure-dominance statistics cover receiver-contact particle
  substeps and use the end-of-step receiver normal. The earlier 45.3% statistic
  concerned a different first-separation cohort and cannot be directly equated.
- Saved world-space FP32 positions slightly perturb support-radius ties in the
  offline neighbor-count comparison; both new cases use the same cache rule.
- Flagged nonconverged acceptance makes data available for inspection; it is not
  evidence of a successful pressure solve or production acceptance.
