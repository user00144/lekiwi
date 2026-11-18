# Mobile Grasping: Coupled Holonomic Base + 5-DoF SO-101 Arm

## Resolving 5-DoF `arm_base` IK Rejections by Stepping the Base

**Trigger:** Inner `EpisodeRunner` stage logs `ik_no_solution` or `swept_path` rejection on a
Cartesian `arm_base` target in `stages/*/events.jsonl`.

**Contrast:** The SO-101 arm has 5 arm DoFs (`shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`) and limited reach. Repeating slightly tweaked XYZ coordinates while
keeping the base stationary burns the action budget; instructing `act()` to adjust base `(x, y, theta)`
when IK rejects brings the object cleanly into the reachable envelope.

```python
grasp = act(
    "Align holonomic base so the red ceramic mug is centered and close in front/wrist views; "
    "if Cartesian arm_base IK rejects reach, step base closer in short guarded moves, grasp, and lift 5 cm.",
    max_actions=14,
    max_steps=28,
    max_seconds=140.0,
)
report["grasp"] = grasp
report["grasp_ok"] = grasp["ok"]
```

**Provenance:** **unverified (reference prose)** — `kinematics.py` + `episode_runtime.py` contract.

## Handling `motion_warning` (`<= 10 deg`) vs. `motion_failure_feedback`

**Trigger:** Stage `events.jsonl` contains `motion_warning` or `reconciliation_attempts > 0`.

**Contrast:**
- `motion_warning` (`<= 10 deg` residual with confirmed hold) is normal under gravity or light table
  contact; `EpisodeRunner` continues from measured joints without entering recovery mode.
- `motion_failure_feedback` (`> 10 deg` incomplete arrival) triggers up to 2 host read-only
  reconciliations and imposes `recovery_constraints` (5 arm joints only, no base or gripper,
  `<= 40 deg` from measured joints, `duration_ms >= 5000`). If an `act()` stage exhausts its budget
  inside `recovery_constraints`, follow with a dedicated retraction stage before retrying the grasp.

```python
if not grasp["ok"] and grasp["reconciliation_attempts"] > 0:
    recover = act(
        "Resolve arm recovery_constraints first by commanding a small safe upward arm-only move (>=5000ms).",
        max_actions=4,
        max_steps=10,
        max_seconds=45.0,
    )
    report["recover"] = recover
```

**Provenance:** **unverified (reference prose)** — `episode.py` recovery profile (`MAX_ARM_WARNING_ERROR_DEG = 10.0`).

## Two-Observation Lift Verification Against False Jaw Closure

**Trigger:** `gripper_moved == True`, yet object slips or jaws closed on empty air.

**Contrast:** Encoder proximity on `gripper` (`0..100` opening percent) never proves physical grasp.
Always phrase `act()` grasp subgoals so the inner loop lifts the object slightly and confirms across
two fresh wrist/front camera observations before claiming `task_complete`.

**Provenance:** **unverified (reference prose)** — `EpisodeRunner` two-observation completion rule.

