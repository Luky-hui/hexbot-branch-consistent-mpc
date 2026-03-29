# OCS2 Hexbot Forward-Progress Root Cause Analysis 2026-03-28

## 1. Executive Summary

The current Hexbot stack has already crossed several earlier blockers that used to dominate the diagnosis:

- `/cmd_vel -> target -> MPC -> backend joint command` is alive.
- Isaac transport is alive.
- the robot can still hold a stable idle stand on the retained mainline.
- gait switching is alive and the moving probe can now sustain a real `21/42` cycle.
- observation whitening and identical-gait republish churn have both been repaired in the retained code.

The live blocker is now narrower:

- the robot can stand
- the target generator can request forward motion
- the gait can cycle
- the solver can stay online without a reset storm
- but the robot still does not convert the moving phase into real physical forward stepping

The current failure is therefore no longer best described as:

- "missing cmd_vel"
- "missing gait"
- "odometry fooled us again"
- "runtime is clipping the whole observation"

The current failure is best described as:

`moving-phase touchdown-anchor drift and swing-joint branch mismatch inside true 21/42 execution`

## 2. Acceptance Target And Current Baselines

Forward-probe acceptance remains:

- `delta_forward_m > 0.05` within the 8 s probe window
- `max_still_sec < 1.0`
- `trunk_sink_detected = false`
- `weird_pose_detected = false`
- tripod gait `21/42` keeps cycling

Two baselines matter at the same time:

### 2.1 Historical stability baseline: `Phase33b`

| Metric | Value | Meaning |
| --- | ---: | --- |
| `delta_forward_m` | `0.004815` | still far below target |
| `max_still_sec` | `3.73` | too long |
| `trunk_sink_detected` | `false` | stable |
| `weird_pose_detected` | `false` | stable |
| gait mode | `21/42` alternating | scheduler alive |

This is still the "do not regress idle/stability" reference.

### 2.2 Current retained-code moving baseline: `codex_gaitdedupe_forward_20260328_194801`

| Metric | Value |
| --- | ---: |
| `initial_policy_received` | `1` |
| `reset_request_sent` | `1` |
| `reject_backend_joint_command` | `0` |
| `runtime_observations_stale` | `0` |
| `delta_forward_m` | `-0.0009468660` |
| `max_still_sec` | `6.666667` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| `mode_sequence` | `[21, 42]` |
| `moving_cycle_detected` | `true` |

Important solver-trace metrics:

| Metric | Value |
| --- | ---: |
| `mode_counts` | `{'21': 6, '42': 6, '63': 6}` |
| `optimized_vs_target_joint_max_abs_delta_mean_rad` | `0.7146` |
| `optimized_vs_current_foot_max_position_error_mean_m` | `0.02936` |
| `current_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.05072` |
| `optimized_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.03259` |

This is the current best retained moving sample because:

- it preserves standing stability
- it keeps the moving gait alive
- it avoids the earlier reset/reject storms
- it makes the remaining blocker easier to isolate

It is not a success case.

## 3. What Has Already Been Repaired

### 3.1 Wrong stationary reference injection

`HexbotCmdVelTargetNode.cpp` no longer pulls the robot back toward a static reference posture whenever `cmd_vel = 0`.

Retained behavior:

- latch live observation joints for stationary target

Practical effect:

- the controller no longer spends idle authority chasing an Isaac-incompatible stand target
- the stable stand baseline is preserved on the current code

### 3.2 Observation whitening / fake-error chain

`HexbotRuntimeMrtNode.cpp` now separates:

- observation normalization
- backend command safety

Retained behavior:

- observation normalization uses Pinocchio model limits
- command safety still uses actuator guarded limits

Practical effect:

- raw observations are no longer prematurely flattened to the guarded command window
- the "state was clipped before MPC even saw it" failure mode has been removed from the live diagnosis

### 3.3 Gait template republish churn

`GaitReceiver.cpp/.h` now deduplicates identical gait templates.

Retained behavior:

- repeated identical templates no longer re-arm `gaitUpdated_`

Practical effect:

- moving probes are no longer dominated by gait reinsertion noise
- `codex_gaitdedupe_forward_20260328_194801_container.log` only reports
  `Setting new gait after time` twice

### 3.4 Moving target continuity

`HexbotCmdVelTargetNode.cpp` also retains:

- mode projection from the active mode schedule template
- a `3.0 s` moving command hold

Practical effect:

- short bridge dropouts no longer immediately collapse the target generator back into idle
- the moving target now survives long enough to expose the real execution defect

## 4. Failed Branches That Were Tested And Reverted

## 4.1 Swing-joint branch penalty inside quadratic tracking cost

Test:

- `codex_swingbranch_forward_20260328_200322`

Intent:

- penalize swing-leg `femur/tarsus` branch discontinuity directly inside `LeggedRobotQuadraticTrackingCost`

Observed result:

| Metric | Value |
| --- | ---: |
| `delta_forward_m` | `-0.0025227602` |
| `max_still_sec` | `8.033334` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |

Comparison against retained mainline:

- branch-guard applications changed from `445` to `450`
- backend rejects stayed at `0`
- gait-template insertions stayed at `2`

Interpretation:

- local tracking cost shaping did not materially reduce real branch flips
- the moving sample got worse
- this branch is rejected and no longer exists in the current code

## 4.2 Global femur software-limit widening

Tests:

- `codex_femurlimit_idle_20260328_201456`
- `codex_femurlimit_forward_20260328_201549`

Intent:

- globally relax the femur software limits used by runtime safety

Observed forward result:

| Metric | Value |
| --- | ---: |
| `initial_policy_received` | `19` |
| `reset_request_sent` | `19` |
| `reject_backend_joint_command` | `18` |
| `delta_forward_m` | `-0.0015540209` |
| `max_still_sec` | `3.933334` |
| `optimized_vs_target_joint_max_abs_delta_mean_rad` | `1.0153` |

Observed command/state split:

- commands reached about `±1.23`
- actual femur states ran to about `±1.37 ~ ±1.45`

Interpretation:

- the current guarded femur window is indeed a software construct, not the URDF hard stop
- but globally widening it is unsafe
- this branch causes a reset/reject storm and is rejected

That branch has also been fully reverted.

## 5. Updated Root-Cause Hierarchy

## 5.1 Background structural causes that still matter

These causes remain true and still explain why the system is fragile:

1. contact-force cost remains expensive relative to horizontal propulsion
2. `reference.info`, `reference_loaded_equilibrium.info`, and Isaac loaded equilibrium are still not perfectly aligned
3. the retained actuator-guard window still constrains the feasible joint corridor more tightly than the URDF range

These are real causes.
They are not, however, the sharpest next debugging handle on the current retained mainline.

## 5.2 The live blocker on the current retained mainline

The active blocker is now concentrated in the true moving phase:

1. touchdown-anchor / stance-world consistency is still poor during `21/42`
2. swing-side `tarsus/femur` joints still jump to the wrong branch inside solver output
3. runtime branch guard can stop the worst flips, but it does not turn them into useful propulsion
4. support-foot execution still does not follow the moving anchor well enough to translate gait cycling into real stepping

This interpretation is supported directly by the retained moving sample:

- moving cycle survives
- reset/reject storm is gone
- target requests forward motion
- yet `current_vs_touchdown_anchor_stance_max_position_error_mean_m` still stays around `5 cm`
- and `optimized_vs_target_joint_max_abs_delta_mean_rad` still stays around `0.71 rad`

So the robot is no longer blocked by missing intent.
It is blocked by moving-phase execution inconsistency.

## 6. What The Root Cause Is No Longer

The primary diagnosis is no longer:

- "the robot has no gait"
- "the controller is dead"
- "the bridge is starving cmd_vel"
- "ROS odometry made up the forward motion"
- "observation clipping is still the main issue"

Those items were either repaired or demoted below the current moving-phase blocker.

## 7. 2026-03-29 Update: Exhaustive Parameter Sweep Confirms Structural Root Cause

### 7.1 Code changes retained from this round

Two code changes were retained on the current mainline (both proven beneficial):

1. **XY stance anchor constraint** in `LeggedRobotPreComputation.cpp` (lines 117-141):
   - `eeZeroVelConConfig` now pins stance feet in world XY using `getPlanarPositionConstraint()` anchor
   - Reduced stance foot drift from 0.054m to 0.023m mean
   - Uses `positionErrorGain` (4.0) as the planar gain
   - Call site changed from `eeZeroVelConConfig()` to `eeZeroVelConConfig(i)`

2. **Measured touchdown anchors** in `HexbotRuntimeMrtNode.cpp` (line 1247):
   - `updateTouchdownAnchors` now uses `currentFootPositions` instead of `targetFootPositions`
   - Anchors are now based on actual measured foot positions at contact transition

### 7.2 Launch parameter additions

`hexbot_sqp.launch.py` now exposes runtime safety thresholds as launch arguments:

| Parameter | Code Default | Current Launch Default |
| --- | --- | --- |
| `commandTrackingResetThreshold` | 1.0 | **2.0** |
| `equivalentAngleWrapResetThreshold` | π (3.14) | **4.5** |
| `enableJointBranchConsistencyGuard` | true | **true** |
| `jointBranchGuardCommandDeviationThreshold` | 0.60 | **1.5** |

### 7.3 Exhaustive parameter sweep results

All tests: `--linear-x 0.03 --angular-z 0.0 --duration 12 --probe-duration 8`

| Config | delta_forward_m | rejects | resets | max_still_sec | safe |
| --- | ---: | ---: | ---: | ---: | --- |
| Baseline (R=5000, guard=on, thresh=default) | -0.001 | 0 | 1 | 6.67 | yes |
| **R=500, guard=on, thresh=relaxed** | **+0.0067** | **0** | **1** | **5.27** | **yes** |
| R=500, guard=off, thresh=relaxed | **+0.034** | 81 | 82 | 4.13 | yes |
| R=500, guard=on, dev_thresh=1.5 | +0.006 | 2 | 3 | 3.43 | yes |
| R=200, guard=on, thresh=relaxed | +0.004 | 11 | 12 | 2.53 | yes |
| R=500, SQP=3, guard=on, thresh=relaxed | +0.0006 | 3 | 4 | 2.73 | yes |
| R=500, guard=on, Q(tarsus)=100 | -0.007 | 2 | 3 | 3.17 | yes |
| R=500, propulsion 4x, thresh=relaxed | -0.002 | 4 | 5 | 5.97 | yes |
| Branch-normalize initializer | -0.002 | 30 | 31 | — | yes |
| R=500, lookahead=0.5s | +0.002 | 0 | 1 | 3.67 | gait=[63] |

### 7.4 Root cause sharpened to π-branch local minimum

The branch guard fires ~470 times in 8s, primarily on:
- `tarsus_joint_LM` (225), `tarsus_joint_LF` (217), `tarsus_joint_RF` (26)

Example from container log:
```
joint=tarsus_joint_LM optimized=0.0493 target=-0.7665 measured=-0.8112
  target_alignment_rad=0.0447 command_deviation_rad=0.8158
```

The solver outputs tarsus at +0.05 while the target is -0.77 (and measured is -0.81).
This is a **π-offset local minimum**, NOT a 2π-wrapping issue.

Key evidence:
- `branchNormalizeJointAngles()` (2π shift) was tested and **made things worse** (30→31 resets)
- The difference between solver output and target is ~0.82 rad (≈π/4 to π/2), not 2π
- Guard-disabled test: `tarsus_joint_RF` tracking_command=-1.3 vs measured=+1.17 (delta=2.4 rad)
- The solver's single SQP linearization step cannot escape this basin

### 7.5 Why guard-off produces 5x more motion

When the guard fires, it replaces the solver's command with the TARGET angle.
For stance tarsus joints, target ≈ measured, so the guard effectively sends "don't move".
Without the guard, wrong-branch commands pass through normalization and create jerky motion,
but some of it averages to net forward progress.

Guard-off rejects come from **tracking threshold** (tarsus delta 2.2–2.4 rad > threshold 2.0),
not normalization threshold.

### 7.6 Conclusion

**Parameter tuning (R/Q weights, safety thresholds, SQP iterations, propulsion force) cannot solve the solver's π-branch convergence problem.**

The next debug pass must address the problem structurally:
- Hard inequality constraints on joint angles per-leg to make wrong-branch infeasible
- Null-space posture penalty that separates left-tarsus-negative from right-tarsus-positive
- Warm-start projection of the solver's previous trajectory onto the correct branch

## 8. Next Action (Updated 2026-03-29)

**DO NOT resume Q/R/threshold tuning. It has been exhaustively proven insufficient.**

The next action must be a **structural constraint-layer fix** at the OCS2 algorithm level:

1. **State inequality constraints** (`StateInputConstraint`):
   - Add per-joint sign constraints: left tarsus ≤ 0, right tarsus ≥ 0
   - Or asymmetric joint limits in the MPC formulation
   - This makes the wrong π-branch mathematically infeasible

2. **Null-space posture cost**:
   - Penalize tarsus angles deviating from their expected sign/range
   - Different from Q-weight tuning: this would be a targeted penalty only on the sign violation

3. **Warm-start branch projection**:
   - After solver produces a solution, project tarsus joints to correct sign before reusing as warm-start
   - Different from the failed `branchNormalizeJointAngles` which only shifted by 2π

Files to audit for constraint implementation:
- `src/ocs2_hexbot_legged_robot/src/LeggedRobotInterface.cpp` — where constraints are registered
- OCS2 upstream `StateInputConstraint` / `StateConstraint` interfaces
- `src/ocs2_hexbot_legged_robot/src/constraint/` directory

Current best retained logs:
- `codex_r500thresh_fwd_20260329_042000` — best safe config (+0.0067m)
- `codex_r500noguard_fwd_20260329_044000` — guard-off proof-of-concept (+0.034m)

This is the most accurate statement of the Hexbot "Stands But Won't Step" root cause as of 2026-03-29.
