# OCS2 Hexbot Bottom-Up Audit 2026-03-28

## 1. Purpose

This audit freezes the current bottom-up algorithm diagnosis after the latest retained-code repairs.

The immediate goal is no longer to explain a generic sink-and-collapse event.
The current goal is to explain why the retained mainline can now:

- hold a stable idle stand
- keep the solver alive
- keep a real `21/42` moving cycle alive
- avoid the earlier observation-whitening and gait-republish traps

and still fail to generate real forward walking.

## 2. Current Audited Conclusion

The diagnosis has changed materially.

The current retained mainline is no longer dominated by:

- dead transport
- dead gait switching
- purely fake observation clipping
- repeated gait-template churn
- a stationary target that keeps pulling the robot toward an Isaac-incompatible stand reference

Those chains were real and have already been cut in the retained code.

The current bottom-up conclusion is:

`the active algorithm-layer blocker now sits inside true moving-phase execution, especially touchdown-anchor consistency and swing-joint branch continuity during 21/42`

## 3. Current Reviewed Modules

This round should be read together with the retained interface-layer changes in:

- `src/ocs2_hexbot_legged_robot_ros/src/HexbotRuntimeMrtNode.cpp`
- `src/ocs2_hexbot_legged_robot_ros/src/HexbotCmdVelTargetNode.cpp`
- `src/ocs2_hexbot_legged_robot_ros/src/gait/GaitReceiver.cpp`
- `src/ocs2_hexbot_legged_robot_ros/include/ocs2_hexbot_legged_robot_ros/gait/GaitReceiver.h`

The next algorithm-layer follow-up is still expected to touch:

- `src/ocs2_hexbot_legged_robot/src/reference_manager/SwitchedModelReferenceManager.cpp`

Supporting configuration and model context:

- `src/ocs2_hexbot_legged_robot/config/mpc/task.info`
- `src/ocs2_hexbot_legged_robot/config/command/reference.info`
- `src/ocs2_hexbot_legged_robot/config/command/reference_loaded_equilibrium.info`
- `src/hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf`

## 4. What Is No Longer The Primary Blocker

### 4.1 Stationary reference mismatch at zero command

`HexbotCmdVelTargetNode.cpp` now latches live observation joints for the stationary target.

This means the current mainline no longer wastes idle authority trying to recover a stale static reference posture whenever `cmd_vel = 0`.

### 4.2 Observation whitening as the dominant runtime defect

`HexbotRuntimeMrtNode.cpp` now splits:

- observation normalization
- backend command safety

Observation normalization uses Pinocchio model limits.
Backend command safety still uses actuator guarded limits.

This means the current mainline no longer feeds the solver a heavily pre-clipped observation as its primary state.

### 4.3 Gait republish churn

`GaitReceiver.cpp/.h` now deduplicates identical gait templates before setting `gaitUpdated_`.

This means repeated template republishes are no longer the main reason the moving probe falls back toward idle behavior.

## 5. Retained Mainline Evidence

The retained moving sample is:

- `codex_gaitdedupe_forward_20260328_194801`

Key facts:

- `initial_policy_received = 1`
- `reset_request_sent = 1`
- `reject_backend_joint_command = 0`
- `runtime_observations_stale = 0`
- `mode_sequence = [21, 42]`
- `moving_cycle_detected = true`
- `delta_forward_m = -0.0009468660`
- `max_still_sec = 6.666667`
- `trunk_sink_detected = false`
- `weird_pose_detected = false`

Key solver-trace metrics:

- `mode_counts = {'21': 6, '42': 6, '63': 6}`
- `optimized_vs_target_joint_max_abs_delta_mean_rad = 0.7146`
- `optimized_vs_current_foot_max_position_error_mean_m = 0.02936`
- `current_vs_touchdown_anchor_stance_max_position_error_mean_m = 0.05072`
- `optimized_vs_touchdown_anchor_stance_max_position_error_mean_m = 0.03259`

Interpretation:

- the solver now survives long enough to expose a real moving-phase defect
- the remaining defect is not "nothing moves"
- the remaining defect is "the moving phase still does not execute coherently enough to create net propulsion"

## 6. Updated Bottom-Layer Defects

### Defect A. Moving-phase touchdown-anchor consistency is still poor

The retained moving sample still shows roughly `5 cm` mean stance-foot error against the touchdown anchor:

- `current_vs_touchdown_anchor_stance_max_position_error_mean_m = 0.05072`

This is too large for the controller to translate a clean gait cycle into stable push-off and body advance.

Operational meaning:

- the stance world anchor is still drifting
- or it is refreshed at the wrong time
- or the execution layer is not staying aligned with the reference anchor through the `63 -> 21 -> 42` transition

### Defect B. Swing-side tarsus/femur branch mismatch still survives into execution

Container logs from the retained mainline still repeatedly report:

- `Applied joint branch-consistency guard`

And the same family of branch flips was still visible when the extra swing-branch penalty was tested.

Operational meaning:

- the solver still explores the wrong kinematic branch during true moving phases
- runtime guard can stop the worst command, but it does not repair the optimization target itself
- wrong-branch avoidance remains a live execution defect, not a solved problem

### Defect C. Moving target generation and moving execution are still not fully aligned

The target layer now correctly:

- projects moving mode from the active mode schedule
- holds the last moving command for `3.0 s`

But the retained moving sample still shows:

- `optimized_vs_target_joint_max_abs_delta_mean_rad = 0.7146`

Operational meaning:

- target intent survives
- yet the optimized joint solution still deviates strongly from the requested moving target
- the most likely remaining interface is the touchdown-anchor / stance-support chain

### Defect D. Guarded IK limits remain part of the moving target corridor

`HexbotCmdVelTargetNode.cpp` still builds its whole-body IK using:

- `getHexbotActuatorGuardedJointLimits(...)`

The current guard values in `HexbotActuatorJointLimits.h` are still:

- femur lower/upper `[-1.00, 1.00]`
- femur guard band `0.05`
- effective guarded window about `[-0.95, 0.95]`

This is not necessarily the next thing to change.
But it remains relevant context because:

- the URDF femur/tarsus ranges are much wider
- the current guarded corridor is software-defined
- any future relaxation must be phase-aware and locally justified, not global

## 7. Rejected Directions

### 7.1 Adding swing-joint branch penalty to quadratic tracking cost

Test:

- `codex_swingbranch_forward_20260328_200322`

Why rejected:

- forward motion got worse
- branch-guard applications stayed essentially unchanged (`445 -> 450`)

Conclusion:

- a generic extra tracking penalty is too weak and too indirect
- it does not solve the moving-phase branch-selection problem

### 7.2 Globally widening femur software limits

Tests:

- `codex_femurlimit_idle_20260328_201456`
- `codex_femurlimit_forward_20260328_201549`

Why rejected:

- idle quality degraded
- moving probe collapsed into a `19/19/18` reset/reject storm

Conclusion:

- this cannot be used as a mainline repair
- any future limit relaxation must be local, phase-aware, and measured-anchor aware

## 8. 2026-03-29 Update: Parametric Sweep Completed — Structural Fix Required

### 8.1 New code retained

1. **XY stance anchor** added to `LeggedRobotPreComputation.cpp` `eeZeroVelConConfig`:
   - stance feet now pinned in world XY via `getPlanarPositionConstraint()` + `positionErrorGain`
   - reduced `current_vs_touchdown_anchor_stance_max_position_error_mean_m` from 0.054 to 0.023

2. **Measured touchdown anchors** in `HexbotRuntimeMrtNode.cpp`:
   - `updateTouchdownAnchors(contactFlags, currentFootPositions)` — uses measured, not target

3. **Launch parameters** for runtime thresholds now exposed in `hexbot_sqp.launch.py`:
   - `commandTrackingResetThreshold=2.0`, `equivalentAngleWrapResetThreshold=4.5`
   - `enableJointBranchConsistencyGuard=true`, `jointBranchGuardCommandDeviationThreshold=1.5`

### 8.2 Rejected directions (2026-03-29)

All of the following were tested and proven insufficient:

| Approach | Test Label | Result | Why Rejected |
| --- | --- | --- | --- |
| R(joint_vel) = 500 | `codex_r500thresh_fwd` | +0.0067m | 13% of target, plateaus |
| R(joint_vel) = 200 | `codex_r200_fwd` | +0.004m, 12 resets | more freedom = more resets |
| SQP iterations = 3 | `codex_r500sqp3_fwd` | +0.0006m | slower solve, same wrong branch |
| Q(tarsus) = 100 | `codex_r500qtarsus_fwd` | -0.007m | over-constrains solver |
| Propulsion force 4x | `codex_r500propf_fwd` | -0.002m, 5 resets | destabilizing |
| Branch-normalize initializer | `codex_branchinit_fwd` | -0.002m, 31 resets | 2π shift ≠ π problem |
| Lookahead 0.5s | `codex_r500la05_fwd` | +0.002m, gait=[63] | gait didn't activate |
| Guard deviation = 1.5 | `codex_r500devthresh_fwd` | +0.006m | marginal |
| Guard disabled | `codex_r500noguard_fwd` | **+0.034m**, 82 resets | proves guard blocks motion |

### 8.3 Root cause sharpened: π-branch local minimum

The solver converges to a **π-offset local minimum** on tarsus joints:

- Left tarsus (LM, LF, LR): measured ≈ -0.8, solver outputs ≈ +0.05 (wrong sign)
- Right tarsus (RF, RM): measured ≈ +1.17, solver outputs ≈ -2.0 (wrong sign after limit clamp)

This is NOT a 2π-wrapping issue. The `branchNormalizeJointAngles` (2π shift) was tested and **made the problem worse**. The solver's single SQP linearization step cannot escape this basin.

The branch guard catches ~470 violations per 8s run. Each time it fires, it replaces the solver command with the target angle (≈ measured), effectively commanding "don't move tarsus". This prevents both danger AND forward progress.

Proof: disabling the guard yields 5x more forward motion (+0.034m vs +0.0067m).

### 8.4 Defect hierarchy update

Previous defects A–D from Section 6 remain valid. A new primary defect supersedes them:

**Defect E (PRIMARY): SQP solver π-branch convergence on tarsus joints**

The solver's cost landscape has a local minimum at the wrong π-branch for tarsus joints. With single-iteration SQP, the linearization at the wrong branch produces gradients that reinforce the wrong solution. The warm-start carries the wrong-branch solution forward across MPC cycles, creating a self-reinforcing loop.

No amount of R/Q/threshold tuning can fix this. The problem requires structural intervention:
1. Hard state inequality constraints making the wrong branch infeasible
2. Null-space posture penalty on tarsus sign violation
3. Solver warm-start projection to correct branch

### 8.5 Next source-reading checklist (updated)

The next pass must audit the **OCS2 constraint registration layer**:

1. `src/ocs2_hexbot_legged_robot/src/LeggedRobotInterface.cpp`
   - where `StateConstraint` / `StateInputConstraint` objects are registered
   - how to add per-joint inequality constraints

2. OCS2 upstream constraint interfaces:
   - `ocs2_core/include/ocs2_core/constraint/StateConstraint.h`
   - `ocs2_core/include/ocs2_core/constraint/StateInputConstraint.h`
   - How soft vs hard inequality constraints work in the SQP solver

3. `src/ocs2_hexbot_legged_robot/src/constraint/` directory
   - existing constraint implementations for reference

4. Latest retained logs:
   - `codex_r500thresh_fwd_20260329_042000` — best safe (+0.0067m)
   - `codex_r500noguard_fwd_20260329_044000` — guard-off proof (+0.034m)

This is the most accurate bottom-up algorithm diagnosis as of 2026-03-29.
