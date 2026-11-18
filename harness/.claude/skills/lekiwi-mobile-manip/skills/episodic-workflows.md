# Episodic Workflows: Coarse Task Progression Patterns

## Multi-Floor Fetch with Spatial Memory Lookup

**Trigger:** Task asks to retrieve or move a named object in a multi-room or multi-floor workspace
when `get_pose()["confidence"] > 0.0`.

**Contrast:** Calling `act("pick up <target>")` directly at trial start fails whenever the robot
starts on another floor or outside camera range; conversely, trusting `search_memory(...)` blindly
and calling `act(...)` immediately on arrival wastes an entire manipulation budget when the stored
record is stale.

```python
import math

report = {"stage": "start"}

# Code block 1: verify localization and query ChromaDB spatial memory.
pose = get_pose()
if pose["confidence"] > 0.0 and not pose["relocalized"]:
    pose = relocalize()
mem = search_memory("red ceramic mug", top_k=3)
report["pose"] = pose
report["hits"] = mem["hits"]
report["stage"] = "memory_queried"

# Code block 2: navigate to top candidate and visually confirm before grasping.
if report["stage"] == "memory_queried" and len(report["hits"]) > 0:
    top_hit = report["hits"][0]
    nav = navigate_to(top_hit["floor"], top_hit["x_m"], top_hit["y_m"], top_hit["yaw_deg"])
    report["nav"] = nav
    if nav["ok"]:
        obs = inspect_scene("Confirm whether the red ceramic mug is visible on the surface.")
        report["inspect"] = obs
        report["stage"] = "confirmed_on_arrival" if obs["ok"] else "inspect_failed"

result = report
```

**Provenance:** **unverified (reference prose)** — baseline architectural stage decomposition;
replace or refine with measured Stage 1 trial provenance during campaigns.

## Stale-Memory Fallback and Map Update Loop

**Trigger:** `inspect_scene(...)` or `act(...)` at a ChromaDB-recalled location confirms the target
object is absent from its previously stored `(floor, x_m, y_m)`.

**Contrast:** Re-querying `search_memory(...)` without invalidating the stale hit returns the exact
same coordinate again on the next block or repeat.

```python
# Inside a guarded recovery block after arrival inspection shows target missing:
if report.get("target_missing") and len(report.get("hits", [])) > 0:
    stale_id = report["hits"][0]["record_id"]
    mark_absent(stale_id, note="surface_empty_on_visual_check")
    next_mem = search_memory("red ceramic mug", top_k=3)
    report["fallback_hits"] = next_mem["hits"]
```

**Provenance:** **unverified (reference prose)** — contract pattern for `SemanticMemoryStore`.

## SLAM-Free Visual Search, Approach, and Grasp Progression

**Trigger:** `get_pose()["confidence"] == 0.0` (global SLAM pose file absent; robot operates purely
from dual OV5648 camera streams).

**Contrast:** Calling `navigate_to(...)` when `confidence == 0.0` immediately returns
`status == "localization_unavailable"`. All search, approach, and grasp stages must run through
`inspect_scene(...)` and `act(...)`.

```python
pose = get_pose()
if pose["confidence"] == 0.0:
    scan = inspect_scene("Sweep view if needed and locate the target object.", max_actions=3, max_steps=8)
    report["scan"] = scan
    if scan["ok"]:
        grasp = act("Drive smoothly into arm reach, grasp the target, and lift slightly to verify hold.")
        report["grasp"] = grasp

```

**Provenance:** **unverified (reference prose)** — visual-only compatibility workflow.

## Mobile Pick-and-Place Completion Gate

**Trigger:** Task requires transporting a grasped object to a destination zone and releasing it.

**Contrast:** Calling `finish(...)` right after `navigate_to(...)` arrives at the destination floor
leaves the object held in the SO-101 jaw instead of placed on the target surface.

```python
if report.get("grasp_ok") and report.get("dest_nav_ok"):
    place = act("Place the held object stably onto the cleared surface, open gripper, and retract safely.")
    report["place"] = place
    if place["ok"]:
        save_memory("red ceramic mug placed on lobby table", room="lobby", tags=["mug", "placed"])
        finish("Delivered and placed red ceramic mug at destination.")
```

**Provenance:** **unverified (reference prose)** — placement completion gate.

