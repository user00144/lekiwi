# LeKiwi Hardware & Runtime Operations

## Guarded Execution Lease and `uncertain_execution` Hard Stop

**Trigger:** `outcome.json` reports `termination_reason == "uncertain_execution"`.

**Contrast:** On LeKiwi, every motor dispatch is bound to a single-use `observation_id` lease and
verified `action_id` receipt between `harness`, `server` (`:5555`/`:5556`), and `lekiwi_runtime` on
Jetson Orin. If transport times out while an `action_id` is in flight, `CampaignPolicyRunner`
aborts `policy.py` immediately. Never wrap or re-run around `uncertain_execution` automatically;
report the halt so physical posture and ZeroMQ link health are checked first.

**Provenance:** **unverified (reference prose)** — `episode.py` & `runner.py` safety invariant.

## Handling `arm_current_out_of_calibration`

**Trigger:** Stage `events.jsonl` shows `error_code == "arm_current_out_of_calibration"` on one
SO-101 joint.

**Contrast:** When a joint's live encoder reading sits outside its calibrated range, changing its
target angle or `duration_ms` cannot pass pre-dispatch validation; that joint must remain `null`
(omitted) while other joints or the holonomic base advance the task safely, or call `block(...)` if
the out-of-range joint is indispensable.

```python
if not manip["ok"] and "calibration" in manip["summary"].lower():
    block("SO-101 joint outside calibrated encoder range; operator inspection required.", stage_summary=manip["summary"])
```

**Provenance:** **unverified (reference prose)** — `episode_runtime.py` calibration restriction rule.

## Dual Camera Roles (`front` vs. `wrist`) and Non-Certifying Sensors

**Trigger:** Interpreting sensor data inside `stages/*/observations/`.

- **Front OV5648 camera**: Room-level landmark orientation, base clearance ahead, coarse approach.
- **Wrist OV5648 camera**: Close-range jaw alignment, contact cues, and post-lift grasp verification.
- **TI AWR2944 radar & BMI160 IMU**: Relative motion / range cues only — radar does not certify
  obstacle-free swept paths, and IMU displacement drifts across origins (`origin_id`).

**Provenance:** **unverified (reference prose)** — `docs/agent-loop.md` sensor contract.

