# Navigation: Relocalization, Multi-Floor Routing, and Visual Handoff

## Relocalizing Ambiguous Start Floor Poses

**Trigger:** `get_pose()` returns `confidence > 0.0` with `relocalized == False` or
`confidence < 0.5`.

**Contrast:** Calling `navigate_to(...)` before relocalization when start floor identity is
uncertain fails with `status == "relocalization_required"` or plans from the wrong floor node.

```python
pose = get_pose()
if pose["confidence"] > 0.0 and (not pose["relocalized"] or pose["confidence"] < 0.5):
    pose = relocalize()
report["pose"] = pose
```

**Provenance:** **unverified (reference prose)** — `TopologicalMap` + `LocalizationProvider` contract.

## Multi-Floor Ramp Traversal via `TopologicalMap`

**Trigger:** Current floor (`pose["floor"]`) differs from target floor (`target_floor`), e.g. `1F`
to `2F` across an indoor ramp.

**Contrast:** Direct Euclidean driving across floors is impossible; `navigate_to(target_floor, x_m, y_m, yaw_deg)`
automatically queries Dijkstra shortest paths across `via="ramp"` edges in `TopologicalMap`.

```python
nav = navigate_to("2F", target_x, target_y, target_yaw)
report["floors_traversed"] = nav["route"]["floors_traversed"]
report["nav_ok"] = nav["ok"]
```

**Provenance:** **unverified (reference prose)** — `TopologicalMap.plan_route` contract.

## Branching Between SLAM Routing and SLAM-Free Visual Driving

**Trigger:** Policy must run seamlessly whether `--pose-file` is active (`confidence > 0`) or
disabled (`confidence == 0.0`).

```python
pose = get_pose()
if pose["confidence"] > 0.0:
    nav = navigate_to(goal_floor, goal_x, goal_y, goal_yaw)
    report["approach_ok"] = nav["ok"]
else:
    vis_nav = act(
        "Visually locate the target using front camera and drive holonomic base smoothly until within arm reach.",
        max_actions=10,
        max_steps=20,
        max_seconds=90.0,
    )
    report["approach_ok"] = vis_nav["ok"]
```

**Provenance:** **unverified (reference prose)** — dual-mode navigation contract.

## Handoff From Coarse Waypoint Arrival to Local Base-Arm Alignment

**Trigger:** `navigate_to` completes (`ok == True`), placing the robot near the support surface,
around `0.5–1.0 m` from the target object.

**Contrast:** Do not hardcode arbitrary metric standoffs (such as fixed centimetre offsets) in
`policy.py` without calibrated depth; instead let `act()` close the final visual gap using live
front and wrist camera feedback alongside holonomic `(x, y, theta)` nudges.

**Provenance:** **unverified (reference prose)** — LeKiwi camera-in-hand handoff design.

